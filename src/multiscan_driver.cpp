#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <vector>
#include <deque>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "util.hpp"
#include "pub_map.hpp"
#include "sick_scan_xd/udp_sockets.h"
#include "sick_scan_xd/msgpack_parser.h"
#include "sick_scan_xd/compact_parser.h"
#include "sick_scan_xd/scansegment_parser_output.h"
#include "sick_scan_xd/sick_scan_common_tcp.h"
#include "sick_scan_xd/sopas_services.h"


class MultiscanNode : public rclcpp::Node
{
public:
    MultiscanNode(bool autostart = true);
    ~MultiscanNode();

    void start();
    void shutdown();

protected:
    void run_receiver();

private:
    static constexpr size_t
        MS100_SEGMENTS_PER_FRAME = 12U,
        MS100_POINTS_PER_SEGMENT_ECHO = 900U,   // points per segment * segments per frame = 10800 points per frame (with 1 echo)
        MS100_MAX_ECHOS_PER_POINT = 3U;         // echos get filterd when we apply different settings in the web dashboard

    struct
    {
        std::string lidar_frame_id;

        std::string lidar_hostname = "";
        std::string driver_hostname = "";
        int lidar_udp_port = 2115;
        // int imu_udp_port = 2115;
        int sopas_tcp_port = 2111;
        bool use_msgpack = false;
        bool use_cola_binary = true;
        double udp_dropout_reset_thresh = 2.;
        double udp_receive_timeout = 1.;
        double sopas_read_timeout = 3.;
        double error_restart_timeout = 3.;
        int max_segment_buffering = 3;
    }
    config;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_pub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;

    sensor_msgs::msg::PointCloud2::_fields_type scan_fields;

    sick_scansegment_xd::UdpReceiverSocketImpl udp_recv_socket;
    std::thread recv_thread;
    std::atomic_bool is_running = true;

};


void swapSegmentsNoIMU(sick_scansegment_xd::ScanSegmentParserOutput& a, sick_scansegment_xd::ScanSegmentParserOutput& b)
{
    std::swap(a.scandata, b.scandata);
    std::swap(a.timestamp, b.timestamp);
    std::swap(a.timestamp_sec, b.timestamp_sec);
    std::swap(a.timestamp_nsec, b.timestamp_nsec);
    std::swap(a.segmentIndex, b.segmentIndex);
    std::swap(a.telegramCnt, b.telegramCnt);
}


MultiscanNode::MultiscanNode(bool autostart) :
    Node("multiscan_driver")
{
    util::declare_param(this, "lidar_frame", this->config.lidar_frame_id, "lidar_link");
    util::declare_param(this, "lidar_hostname", this->config.lidar_hostname, "");
    util::declare_param(this, "driver_hostname", this->config.driver_hostname, "");
    util::declare_param(this, "lidar_udp_port", this->config.lidar_udp_port, 2115);
    // util::declare_param(this, "imu_udp_port", this->config.imu_udp_port, 2115);
    util::declare_param(this, "sopas_tcp_port", this->config.sopas_tcp_port, 2111);
    util::declare_param(this, "use_msgpack", this->config.use_msgpack, false);
    util::declare_param(this, "use_cola_binary", this->config.use_cola_binary, true);
    util::declare_param(this, "udp_reset_timeout", this->config.udp_dropout_reset_thresh, 2.);
    util::declare_param(this, "udp_receive_timeout", this->config.udp_receive_timeout, 1.);
    util::declare_param(this, "sopas_read_timeout", this->config.sopas_read_timeout, 3.);
    util::declare_param(this, "error_restart_timeout", this->config.error_restart_timeout, 3.);
    util::declare_param(this, "max_segment_buffers", this->config.max_segment_buffering, 3);

    this->scan_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_scan", rclcpp::SensorDataQoS{});
    this->imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("lidar_imu", rclcpp::SensorDataQoS{});

    this->scan_fields = {
        sensor_msgs::msg::PointField{}
            .set__name("x")
            .set__datatype(sensor_msgs::msg::PointField::FLOAT32)
            .set__count(1)
            .set__offset(0),
        sensor_msgs::msg::PointField{}
            .set__name("y")
            .set__datatype(sensor_msgs::msg::PointField::FLOAT32)
            .set__count(1)
            .set__offset(4),
        sensor_msgs::msg::PointField{}
            .set__name("z")
            .set__datatype(sensor_msgs::msg::PointField::FLOAT32)
            .set__count(1)
            .set__offset(8),
        sensor_msgs::msg::PointField{}
            .set__name("i")
            .set__datatype(sensor_msgs::msg::PointField::FLOAT32)
            .set__count(1)
            .set__offset(12),
        sensor_msgs::msg::PointField{}
            .set__name("range")
            .set__datatype(sensor_msgs::msg::PointField::FLOAT32)
            .set__count(1)
            .set__offset(16),
        sensor_msgs::msg::PointField{}
            .set__name("azimuth")
            .set__datatype(sensor_msgs::msg::PointField::FLOAT32)
            .set__count(1)
            .set__offset(20),
        sensor_msgs::msg::PointField{}
            .set__name("elevation")
            .set__datatype(sensor_msgs::msg::PointField::FLOAT32)
            .set__count(1)
            .set__offset(24),
        sensor_msgs::msg::PointField{}
            .set__name("layer")
            .set__datatype(sensor_msgs::msg::PointField::UINT32)
            .set__count(1)
            .set__offset(28),
        sensor_msgs::msg::PointField{}
            .set__name("echo")
            .set__datatype(sensor_msgs::msg::PointField::UINT32)
            .set__count(1)
            .set__offset(32),
        sensor_msgs::msg::PointField{}
            .set__name("index")
            .set__datatype(sensor_msgs::msg::PointField::UINT32)
            .set__count(1)
            .set__offset(36),
        sensor_msgs::msg::PointField{}
            .set__name("t")
            .set__datatype(sensor_msgs::msg::PointField::FLOAT32)
            .set__count(1)
            .set__offset(40)
    };

    if(autostart)
    {
        this->start();
    }
}

MultiscanNode::~MultiscanNode()
{
    this->shutdown();
}

void MultiscanNode::start()
{
    if(!this->recv_thread.joinable())
    {
        this->is_running = true;
        this->recv_thread = std::thread{ &MultiscanNode::run_receiver, this };
    }
}

void MultiscanNode::run_receiver()
{
    while(this->is_running)
    {
        RCLCPP_INFO(this->get_logger(),
            "[MULTISCAN DRIVER]: Initializing connections using the following parameters:"
            "\n\tLidar IP address: %s"
            "\n\tDriver IP address: %s"
            "\n\tLidar UDP port: %d"
            "\n\tSOPAS TCP port: %d"
            "\n\tData format: %s"
            "\n\tCoLa configuration: %s",
            this->config.lidar_hostname.c_str(),
            this->config.driver_hostname.c_str(),
            this->config.lidar_udp_port,
            this->config.sopas_tcp_port,
            this->config.use_msgpack ? "MsgPack" : "Compact",
            this->config.use_cola_binary ? "Binary" : "ASCII");

        if(this->udp_recv_socket.Init(/*this->config.lidar_hostname*/ "", this->config.lidar_udp_port))
        {
            RCLCPP_INFO(this->get_logger(), "[MULTISCAN DRIVER]: UDP socket created successfully");

            sick_scan_xd::SickScanCommonTcp sopas_tcp{
                this->config.lidar_hostname, this->config.sopas_tcp_port, this->config.use_cola_binary ? 'B' : 'A' };
            sick_scan_xd::SopasServices sopas_service{ &sopas_tcp, this->config.use_cola_binary };
            sopas_tcp.init_device();    // TODO: can block indefinitely with valid config that doesn't actually exist
            sopas_tcp.setReadTimeOutInMs(static_cast<size_t>(this->config.sopas_read_timeout * 1e3));

            if(sopas_tcp.isConnected())
            {
                RCLCPP_INFO(this->get_logger(), "[MULTISCAN DRIVER]: TCP connected! Sending startup commands...");
                sopas_service.sendAuthorization();
                sopas_service.sendMultiScanStartCmd(
                    this->config.driver_hostname,
                    this->config.lidar_udp_port,
                    (2 - this->config.use_msgpack),
                    true,
                    this->config.lidar_udp_port);
                RCLCPP_INFO(this->get_logger(), "[MULTISCAN DRIVER]: Successfully sent all startup commands. Proceeding to UDP decode loop.");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "[MULTISCAN DRIVER]: TCP not connected! Could not send SOPAS initialization command!");
            }

            constexpr size_t RECV_BUFFER_SIZE = 64 * 1024;
            std::vector<uint8_t>
                udp_buffer(RECV_BUFFER_SIZE, 0),
                udp_msg_start_seq({ 0x02, 0x02,  0x02,  0x02 });
            double udp_recv_timeout = -1.;
            chrono_system_time timestamp_last_udp_recv = chrono_system_clock::now();
            std::array<std::deque<sick_scansegment_xd::ScanSegmentParserOutput>, MS100_SEGMENTS_PER_FRAME> samples{};
            size_t filled_segments = 0;

            try
            {
                while(this->is_running && sopas_tcp.isConnected())
                {
                    size_t bytes_received = this->udp_recv_socket.Receive(udp_buffer, udp_recv_timeout, udp_msg_start_seq);
                    // RCLCPP_INFO(this->get_logger(), "[MULTISCAN DRIVER]: Received %ld bytes from %d", bytes_received, this->udp_recv_socket.port());
                    if( bytes_received > udp_msg_start_seq.size() + 8 &&
                        std::equal(udp_buffer.begin(), udp_buffer.begin() + udp_msg_start_seq.size(), udp_msg_start_seq.begin()) )
                    {
                        uint32_t payload_length_bytes = 0;
                        uint32_t bytes_to_receive = 0;
                        uint32_t udp_payload_offset = 0;

                        if(this->config.use_msgpack)
                        {
                            payload_length_bytes = sick_scansegment_xd::Convert4Byte(udp_buffer.data() + udp_msg_start_seq.size());
                            bytes_to_receive = (uint32_t)(payload_length_bytes + udp_msg_start_seq.size() + 2 * sizeof(uint32_t));
                            udp_payload_offset = udp_msg_start_seq.size() + sizeof(uint32_t); // payload starts after (4 byte \x02\x02\x02\x02) + (4 byte payload length)
                        }
                        else
                        {
                            bool parse_success = false;
                            uint32_t num_bytes_required = 0;
                            chrono_system_time recv_start_timestamp = chrono_system_clock::now();
                            while (this->is_running &&
                                (parse_success = sick_scansegment_xd::CompactDataParser::ParseSegment(udp_buffer.data(), bytes_received, 0, payload_length_bytes, num_bytes_required )) == false &&
                                (udp_recv_timeout < 0 || sick_scansegment_xd::Seconds(recv_start_timestamp, chrono_system_clock::now()) < udp_recv_timeout)) // read blocking (udp_recv_timeout < 0) or udp_recv_timeout in seconds
                            {
                                if(num_bytes_required > 1024 * 1024)
                                {
                                    parse_success = false;
                                    // RCLCPP_INFO(this->get_logger(), "[MULTISCAN DRIVER]: Received %ld bytes (compact), %lu bytes required - probably incorrect payload.", bytes_received, num_bytes_required + sizeof(uint32_t));
                                    sick_scansegment_xd::CompactDataParser::ParseSegment(udp_buffer.data(), bytes_received, 0, payload_length_bytes, num_bytes_required , 0.0f, 1); // parse again with debug output after error
                                    break;
                                }
                                // RCLCPP_INFO(this->get_logger(), "[MULTISCAN DRIVER]: %ld bytes received (compact), %lu bytes or more required.", bytes_received, num_bytes_required + sizeof(uint32_t));
                                while(this->is_running && bytes_received < num_bytes_required + sizeof(uint32_t) && // payload + 4 byte CRC required
                                    (udp_recv_timeout < 0 || sick_scansegment_xd::Seconds(recv_start_timestamp, chrono_system_clock::now()) < udp_recv_timeout)) // read blocking (udp_recv_timeout < 0) or udp_recv_timeout in seconds
                                {
                                    std::vector<uint8_t> chunk_buffer(RECV_BUFFER_SIZE, 0);
                                    size_t chunk_bytes_received = this->udp_recv_socket.Receive(chunk_buffer);
                                    // RCLCPP_INFO(this->get_logger(), "[MULTISCAN DRIVER]: Received chunk of %ld bytes.", chunk_bytes_received);
                                    udp_buffer.insert(udp_buffer.begin() + bytes_received, chunk_buffer.begin(), chunk_buffer.begin() + chunk_bytes_received);
                                    bytes_received += chunk_bytes_received;
                                }
                            }
                            if(!parse_success)
                            {
                                RCLCPP_INFO(this->get_logger(), "[MULTISCAN DRIVER]: Compact payload parse failed.");
                                continue;
                            }
                            bytes_to_receive = (uint32_t)(payload_length_bytes + sizeof(uint32_t)); // payload + (4 byte CRC)
                            udp_payload_offset = 0; // compact format calculates CRC over complete message (incl. header)
                            // RCLCPP_INFO(this->get_logger(), "[MULTISCAN DRIVER]: Payload bytes: %ld, Bytes to receive: %lu, Bytes received: %ld", payload_length_bytes, bytes_to_receive, bytes_received);
                        }

                        // if(bytes_received != bytes_to_receive)
                        // {
                        //     RCLCPP_INFO(this->get_logger(), "[MULTISCAN DRIVER]: ERROR: Recieved %ld bytes but expected %lu bytes!", bytes_received, bytes_to_receive);
                        // }

                        size_t bytes_valid = std::min<size_t>(bytes_received, (size_t)bytes_to_receive);
                        uint32_t u32PayloadCRC = sick_scansegment_xd::Convert4Byte(udp_buffer.data() + bytes_valid - sizeof(uint32_t)); // last 4 bytes are CRC
                        std::vector<uint8_t> msgpack_payload{ udp_buffer.begin() + udp_payload_offset, udp_buffer.begin() + bytes_valid - sizeof(uint32_t) };
                        uint32_t u32MsgPackCRC = sick_scansegment_xd::crc32(0, msgpack_payload.data(), msgpack_payload.size());

                        if(u32PayloadCRC != u32MsgPackCRC)
                        {
                            RCLCPP_INFO(this->get_logger(), "[MULTISCAN DRIVER]: CRC payload check failed.");
                            continue;
                        }

                        // process
                        {
                            sick_scansegment_xd::ScanSegmentParserOutput segment;
                            if(this->config.use_msgpack)
                            {
                                if(!sick_scansegment_xd::MsgPackParser::Parse(udp_buffer, fifo_clock::now(), segment, true, false))
                                {
                                    RCLCPP_INFO(this->get_logger(), "[MULTISCAN DRIVER]: Msgpack parse failed.");
                                    continue;
                                }
                            }
                            else
                            {
                                if(!sick_scansegment_xd::CompactDataParser::Parse(udp_buffer, fifo_clock::now(), segment, 0, true, false))
                                {
                                    RCLCPP_INFO(this->get_logger(), "[MULTISCAN DRIVER]: Compact parse failed.");
                                    continue;
                                }
                            }

                            // export imu if available
                            if(segment.imudata.valid)
                            {
                                sensor_msgs::msg::Imu msg;

                                msg.header.stamp.sec = segment.timestamp_sec;
                                msg.header.stamp.nanosec = segment.timestamp_nsec;
                                msg.header.frame_id = this->config.lidar_frame_id;

                                msg.angular_velocity.x = segment.imudata.angular_velocity_x;
                                msg.angular_velocity.y = segment.imudata.angular_velocity_y;
                                msg.angular_velocity.z = segment.imudata.angular_velocity_z;

                                msg.linear_acceleration.x = segment.imudata.acceleration_x;
                                msg.linear_acceleration.y = segment.imudata.acceleration_y;
                                msg.linear_acceleration.z = segment.imudata.acceleration_z;

                                msg.orientation.w = segment.imudata.orientation_w;
                                msg.orientation.x = segment.imudata.orientation_x;
                                msg.orientation.y = segment.imudata.orientation_y;
                                msg.orientation.z = segment.imudata.orientation_z;

                                this->imu_pub->publish(msg);
                            }

                            if(segment.scandata.size() > 0)
                            {
                                const size_t idx = segment.segmentIndex;
                                samples[idx].emplace_front();
                                samples[idx].resize(this->config.max_segment_buffering);
                                swapSegmentsNoIMU(samples[idx].front(), segment);
                                filled_segments |= 1 << idx;
                            }

                            if(filled_segments >= (1 << MS100_SEGMENTS_PER_FRAME) - 1)
                            {
                                // assemble and publish pc
                                sensor_msgs::msg::PointCloud2 scan;
                                constexpr size_t MS100_NOMINAL_POINTS_PER_SCAN = MS100_POINTS_PER_SEGMENT_ECHO * MS100_SEGMENTS_PER_FRAME;  // single echo
                                scan.data.reserve(MS100_NOMINAL_POINTS_PER_SCAN * 44);  // 44 bytes per point
                                scan.data.resize(0);

                                uint64_t earliest_ts = 0xFFFFFFFFFFFFFFFFUL;
                                for(auto& segment_queue : samples)
                                {
                                    const auto& _seg = segment_queue.front();
                                    uint64_t ts = _seg.timestamp_sec * 1000000000 + _seg.timestamp_nsec;
                                    if(ts < earliest_ts) earliest_ts = ts;
                                    for(const auto& _group : _seg.scandata)
                                    {
                                        for(const auto& _line : _group.scanlines)
                                        {
                                            for(const auto& _point : _line.points)
                                            {
                                                scan.data.resize(scan.data.size() + 44);
                                                uint8_t* _point_data = scan.data.end().base() - 44;
                                                memcpy(_point_data, &_point, 40);
                                                reinterpret_cast<float*>(_point_data)[11] = 0.f;
                                            }
                                        }
                                    }
                                    segment_queue.clear();
                                }

                                scan.fields = this->scan_fields;
                                scan.is_bigendian = false;
                                scan.point_step = 44;
                                scan.row_step = scan.data.size();
                                scan.height = 1;
                                scan.width = scan.data.size() / 44;
                                scan.is_dense = true;
                                scan.header.frame_id = this->config.lidar_frame_id;
                                scan.header.stamp.sec = earliest_ts / 1000000000;
                                scan.header.stamp.nanosec = earliest_ts % 1000000000;

                                this->scan_pub->publish(scan);
                                filled_segments = 0;
                            }
                        }

                        if(bytes_received > 0)
                        {
                            timestamp_last_udp_recv = chrono_system_clock::now();
                        }
                        if(sick_scansegment_xd::Seconds(timestamp_last_udp_recv, chrono_system_clock::now()) > this->config.udp_dropout_reset_thresh)
                        {
                            udp_recv_timeout = -1;
                        }
                        else
                        {
                            udp_recv_timeout = this->config.udp_receive_timeout; // receive non-blocking with timeout
                        }
                    }
                }

                if(!sopas_tcp.isConnected())
                {
                    RCLCPP_INFO(this->get_logger(), "[MULTISCAN DRIVER]: SOPAS TCP connection lost - restarting...");
                }
            }
            catch(const std::exception& e)
            {
                RCLCPP_INFO(this->get_logger(), "[MULTISCAN DRIVER]: UDP decode loop encountered an exception - what():\n\t%s", e.what());
            }

            if(sopas_tcp.isConnected())
            {
                sopas_service.sendAuthorization();
                sopas_service.sendMultiScanStopCmd(true);
            }
        }

        if(this->is_running)
        {
            RCLCPP_INFO(this->get_logger(), "[MULTISCAN DRIVER]: Encountered error - restarting after timeout...");
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<size_t>(this->config.error_restart_timeout * 1e3)));
        }
    }
}

void MultiscanNode::shutdown()
{
    if(this->is_running || this->recv_thread.joinable())
    {
        this->is_running = false;
        this->udp_recv_socket.ForceStop();
        this->recv_thread.join();
    }
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiscanNode>());
    rclcpp::shutdown();

    return 0;
}
