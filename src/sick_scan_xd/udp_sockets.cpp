#include "udp_sockets.h"
#include "sick_ros_wrapper.h"


sick_scansegment_xd::UdpReceiverSocketImpl::UdpReceiverSocketImpl() :
    m_udp_sender(""), m_udp_port(0), m_udp_socket(INVALID_SOCKET), m_force_quit(false) {}

sick_scansegment_xd::UdpReceiverSocketImpl::~UdpReceiverSocketImpl()
{
    try
    {
        if (m_udp_socket != INVALID_SOCKET)
        {
            // shutdown(m_udp_socket, SHUT_RDWR);
            closesocket(m_udp_socket);
            m_udp_socket = INVALID_SOCKET;
        }
    }
    catch (std::exception & e)
    {
        ROS_ERROR_STREAM("## ERROR ~UdpReceiverSocketImpl: can't close socket, " << e.what());
    }
}

bool sick_scansegment_xd::UdpReceiverSocketImpl::Init(const std::string& udp_sender, int udp_port)
{
    try
    {
        if(m_udp_socket != INVALID_SOCKET)
        {
            ForceStop();
        }

        wsa_init();
        m_udp_sender = udp_sender;
        m_udp_port = udp_port;
        m_udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (m_udp_socket == INVALID_SOCKET)
        {
            ROS_ERROR_STREAM("## ERROR UdpReceiverSocketImpl::Init(" << m_udp_sender << ":" << m_udp_port << "): can't open socket, error: " << getErrorMessage());
            return false;
        }
        // #if defined WIN32 || defined _MSC_VER
        // char broadcast_opt = 1, reuse_addr_opt = 1;
        // #else
        // int broadcast_opt = 1, reuse_addr_opt = 1;
        // #endif
        // setsockopt(m_udp_socket, SOL_SOCKET, SO_BROADCAST, &broadcast_opt, sizeof(broadcast_opt));
        // setsockopt(m_udp_socket, SOL_SOCKET, SO_REUSEADDR, &reuse_addr_opt, sizeof(reuse_addr_opt));
        struct sockaddr_in sim_servaddr = { 0 };
        if(m_udp_sender.empty())
            sim_servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
        else
            sim_servaddr.sin_addr.s_addr = inet_addr(m_udp_sender.c_str()); 
        sim_servaddr.sin_family = AF_INET;
        sim_servaddr.sin_port = htons(m_udp_port);
        ROS_INFO_STREAM("UdpReceiverSocketImpl: udp socket created, binding to port " << ntohs(sim_servaddr.sin_port) << " ... ");
        if (bind(m_udp_socket, (SOCKADDR*)&sim_servaddr, sizeof(sim_servaddr)) < 0)
        {
            ROS_ERROR_STREAM("## ERROR UdpReceiverSocketImpl::Init(" << m_udp_sender << ":" << m_udp_port << "): can't bind socket, error: " << getErrorMessage());
            closesocket(m_udp_socket);
            m_udp_socket = INVALID_SOCKET;
            return false;
        }
        return true;
    }
    catch (std::exception & e)
    {
        m_udp_socket = INVALID_SOCKET;
        ROS_ERROR_STREAM("## ERROR UdpReceiverSocketImpl::Init(): can't open socket to " << m_udp_sender << ":" << m_udp_port << ", exception: " << e.what());
        return false;
    }
}

void sick_scansegment_xd::UdpReceiverSocketImpl::ForceStop()
{
    m_force_quit = true;
#if defined WIN32 || defined _MSC_VER
    closesocket(m_udp_socket);
#else
    shutdown(m_udp_socket, SHUT_RDWR);
    closesocket(m_udp_socket);
#endif
    m_udp_socket = INVALID_SOCKET;
}

size_t sick_scansegment_xd::UdpReceiverSocketImpl::Receive(std::vector<uint8_t>& msg_payload)
{
    int64_t bytes_received = 0;
    while(!m_force_quit && bytes_received == 0)
        bytes_received = recv(m_udp_socket, (char*)msg_payload.data(), (int)msg_payload.size(), 0);
    if (bytes_received < 0)
        return 0; // socket error
    return (size_t)bytes_received;
}

size_t sick_scansegment_xd::UdpReceiverSocketImpl::Receive(std::vector<uint8_t>& msg_payload, double timeout, const std::vector<uint8_t>& udp_msg_start_seq)
{
    chrono_system_time start_timestamp = chrono_system_clock::now();
    size_t headerlength = udp_msg_start_seq.size() + sizeof(uint32_t); // 8 byte header: 0x02020202 + Payloadlength
    size_t bytes_received = 0;
    size_t bytes_to_receive = msg_payload.size();
    // Receive \x02\x02\x02\x02 | 4Bytes payloadlength incl. CRC | Payload | CRC32
    while (!m_force_quit && bytes_received < bytes_to_receive && (timeout < 0 || sick_scansegment_xd::Seconds(start_timestamp, chrono_system_clock::now()) < timeout))
    {
        int64_t chunk_bytes_received = recv(m_udp_socket, (char*)msg_payload.data() + bytes_received, (int)msg_payload.size() - bytes_received, 0);
        if (chunk_bytes_received <= 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        // std::cout << "UdpSenderSocketImpl::Receive(): chunk of " << std::dec << chunk_bytes_received << " bytes received: " << std::endl;
        // for(int n = 0; n < chunk_bytes_received; n++)
        //     std::cout << std::setfill('0') << std::setw(2) << std::hex << (int)(msg_payload.data()[bytes_received + n] & 0xFF);
        // std::cout << std::endl;
        if (bytes_received == 0 && chunk_bytes_received > (int64_t)headerlength && std::equal(msg_payload.begin(), msg_payload.begin() + udp_msg_start_seq.size(), udp_msg_start_seq.begin())) // start of new msgpack
        {
            // Start of new message: restart timeout
            start_timestamp = chrono_system_clock::now();
            // Decode 8 byte header: 0x02020202 + Payloadlength
            size_t Payloadlength= Convert4Byte(msg_payload.data() + udp_msg_start_seq.size());
            bytes_to_receive = Payloadlength + headerlength + sizeof(uint32_t); // 8 byte header + payload + 4 byte CRC
            if(bytes_to_receive > msg_payload.size())
            {
                ROS_ERROR_STREAM("## ERROR UdpReceiverSocketImpl::Receive(): unexpected payloadlength " << Payloadlength << " byte incl CRC received");
                break;
            }
            bytes_received += chunk_bytes_received;
        }
        else if (bytes_received > 0) // continue receiving a msgpack
        {
            bytes_received += chunk_bytes_received;
        }
    }
    return bytes_received;
}


sick_scansegment_xd::UdpSenderSocketImpl::UdpSenderSocketImpl(const std::string& server_address, int udp_port) :
    m_socket_opened(false), m_udp_socket(INVALID_SOCKET)
{
    try
    {
        m_server_address = server_address;
        m_udp_port = udp_port;
        if ((m_udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == INVALID_SOCKET)
        {
            ROS_ERROR_STREAM("## ERROR UdpSenderSocketImpl::init(" << server_address << ":" << udp_port << "): can't create socket, error: " << getErrorMessage());
        }
        else
        {
            #if defined WIN32 || defined _MSC_VER
            char broadcast_opt = 1; // reuse_addr_opt = 1
            #else
            int broadcast_opt = 1; // reuse_addr_opt = 1
            #endif
            if (setsockopt(m_udp_socket, SOL_SOCKET, SO_BROADCAST, &broadcast_opt, sizeof(broadcast_opt)) < 0)
            {
                ROS_ERROR_STREAM("## ERROR UdpSenderSocketImpl::init(" << server_address << ":" << udp_port << "): setsockopt(SO_BROADCAST) failed, error: " << getErrorMessage());
            }
            // setsockopt(m_udp_socket, SOL_SOCKET, SO_REUSEADDR, &reuse_addr_opt, sizeof(reuse_addr_opt));
        }
    }
    catch (const std::exception& e)
    {
        m_udp_socket = INVALID_SOCKET;
        ROS_ERROR_STREAM("## ERROR UdpSenderSocketImpl(): socket initialization failed, exception: " << e.what());
    }
}

sick_scansegment_xd::UdpSenderSocketImpl::~UdpSenderSocketImpl()
{
    try
    {
        if (m_udp_socket != INVALID_SOCKET)
        {
            // shutdown(m_udp_socket, SHUT_RDWR);
            closesocket(m_udp_socket);
            m_udp_socket = INVALID_SOCKET;
        }
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("## ERROR ~UdpSenderSocketImpl(): socket shutdown and close failed, exception: " << e.what());
    }
}

bool sick_scansegment_xd::UdpSenderSocketImpl::Send(std::vector<uint8_t>& message)
{
    size_t bytes_sent = 0;
    if (m_udp_socket != INVALID_SOCKET)
    {
        try
        {
            struct sockaddr_in sim_servaddr = { 0 };
            if(m_server_address.empty())
            {
                sim_servaddr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
            }
            else
            {
                #if defined WIN32 || defined _MSC_VER
                sim_servaddr.sin_addr.s_addr = inet_addr(m_server_address.c_str());
                #else
                struct in_addr sim_in_addr;
                if (inet_aton(m_server_address.c_str(), &sim_in_addr) != 0)
                {
                    sim_servaddr.sin_addr.s_addr = sim_in_addr.s_addr;
                }
                else
                {
                    ROS_ERROR_STREAM("## ERROR UdpSenderSocketImpl()::Send(): inet_aton(" << m_server_address << ") failed (invalid address)");
                    sim_servaddr.sin_addr.s_addr = inet_addr(m_server_address.c_str());
                }
                #endif
            }
            sim_servaddr.sin_family = AF_INET;
            sim_servaddr.sin_port = htons(m_udp_port);
            bytes_sent = sendto(m_udp_socket, (const char*)message.data(), message.size(), 0, (SOCKADDR*)&sim_servaddr, sizeof(sim_servaddr));
            if (bytes_sent != message.size())
            {
                ROS_ERROR_STREAM("## ERROR UdpSenderSocketImpl()::Send() failed, " << bytes_sent << " of " << message.size() << " bytes sent.");
            }
        }
        catch (const std::exception& e)
        {
            ROS_ERROR_STREAM("## ERROR UdpSenderSocketImpl()::Send() failed, exception: " << e.what());
        }
    }
    else
    {
        ROS_ERROR_STREAM("## ERROR UdpSenderSocketImpl()::Send(): udp socket not initialized");
    }
    return (bytes_sent == message.size());
}
