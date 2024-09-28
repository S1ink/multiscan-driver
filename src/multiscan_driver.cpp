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
        int lidar_udp_port = 2115;
        bool use_msgpack = false;
        double udp_dropout_reset_thresh = 2.;
        double udp_receive_timeout = 1.;
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
    util::declare_param(this, "lidar_udp_port", this->config.lidar_udp_port, 2115);
    util::declare_param(this, "use_msgpack", this->config.use_msgpack, false);
    util::declare_param(this, "udp_reset_timeout", this->config.udp_dropout_reset_thresh, 2.);
    util::declare_param(this, "udp_receive_timeout", this->config.udp_receive_timeout, 1.);
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
            .set__offset(0),
        sensor_msgs::msg::PointField{}
            .set__name("z")
            .set__datatype(sensor_msgs::msg::PointField::FLOAT32)
            .set__count(1)
            .set__offset(0),
        sensor_msgs::msg::PointField{}
            .set__name("i")
            .set__datatype(sensor_msgs::msg::PointField::FLOAT32)
            .set__count(1)
            .set__offset(0),
        sensor_msgs::msg::PointField{}
            .set__name("range")
            .set__datatype(sensor_msgs::msg::PointField::FLOAT32)
            .set__count(1)
            .set__offset(0),
        sensor_msgs::msg::PointField{}
            .set__name("azimuth")
            .set__datatype(sensor_msgs::msg::PointField::FLOAT32)
            .set__count(1)
            .set__offset(0),
        sensor_msgs::msg::PointField{}
            .set__name("elevation")
            .set__datatype(sensor_msgs::msg::PointField::FLOAT32)
            .set__count(1)
            .set__offset(0),
        sensor_msgs::msg::PointField{}
            .set__name("layer")
            .set__datatype(0)
            .set__count(1)
            .set__offset(0),
        sensor_msgs::msg::PointField{}
            .set__name("echo")
            .set__datatype(0)
            .set__count(1)
            .set__offset(0),
        sensor_msgs::msg::PointField{}
            .set__name("point")
            .set__datatype(0)
            .set__count(1)
            .set__offset(0),
        sensor_msgs::msg::PointField{}
            .set__name("t")
            .set__datatype(sensor_msgs::msg::PointField::UINT32)
            .set__count(1)
            .set__offset(0),
        sensor_msgs::msg::PointField{}
            .set__name("reflector")
            .set__datatype(0)
            .set__count(1)
            .set__offset(0),
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
        if(this->udp_recv_socket.Init(this->config.lidar_hostname, this->config.lidar_udp_port))
        {
            // SOPAS start command

            constexpr size_t RECV_BUFFER_SIZE = 64 * 1024;
            std::vector<uint8_t>
                udp_buffer(RECV_BUFFER_SIZE, 0),
                chunk_buffer(RECV_BUFFER_SIZE, 0),
                udp_msg_start_seq({ 0x02, 0x02,  0x02,  0x02 });
            double udp_recv_timeout = -1.;
            chrono_system_time timestamp_last_udp_recv = chrono_system_clock::now();
            std::array<std::deque<sick_scansegment_xd::ScanSegmentParserOutput>, MS100_SEGMENTS_PER_FRAME> samples{};
            size_t filled_segments = 0;
            while(this->is_running)
            {
                size_t bytes_received = this->udp_recv_socket.Receive(udp_buffer, udp_recv_timeout, udp_msg_start_seq);
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
                                sick_scansegment_xd::CompactDataParser::ParseSegment(udp_buffer.data(), bytes_received, 0, payload_length_bytes, num_bytes_required , 0.0f, 1); // parse again with debug output after error
                                break;
                            }
                            while(this->is_running && bytes_received < num_bytes_required + sizeof(uint32_t) && // payload + 4 byte CRC required
                                (udp_recv_timeout < 0 || sick_scansegment_xd::Seconds(recv_start_timestamp, chrono_system_clock::now()) < udp_recv_timeout)) // read blocking (udp_recv_timeout < 0) or udp_recv_timeout in seconds
                            {
                                size_t chunk_bytes_received = this->udp_recv_socket.Receive(chunk_buffer);
                                udp_buffer.insert(udp_buffer.begin() + bytes_received, chunk_buffer.begin(), chunk_buffer.begin() + chunk_bytes_received);
                                bytes_received += chunk_bytes_received;
                            }
                        }
                        if(!parse_success) continue;
                        bytes_to_receive = (uint32_t)(payload_length_bytes + sizeof(uint32_t)); // payload + (4 byte CRC)
                        udp_payload_offset = 0; // compact format calculates CRC over complete message (incl. header)
                    }

                    size_t bytes_valid = std::min<size_t>(bytes_received, (size_t)bytes_to_receive);
                    uint32_t u32PayloadCRC = sick_scansegment_xd::Convert4Byte(udp_buffer.data() + bytes_valid - sizeof(uint32_t)); // last 4 bytes are CRC
                    std::vector<uint8_t> msgpack_payload{ udp_buffer.begin() + udp_payload_offset, udp_buffer.begin() + bytes_valid - sizeof(uint32_t) };
                    uint32_t u32MsgPackCRC = sick_scansegment_xd::crc32(0, msgpack_payload.data(), msgpack_payload.size());

                    if(u32PayloadCRC != u32MsgPackCRC) continue;

                    // process
                    {
                        sick_scansegment_xd::ScanSegmentParserOutput segment;
                        if(this->config.use_msgpack)
                        {
                            sick_scansegment_xd::MsgPackParser::Parse(udp_buffer, fifo_clock::now(), segment, true, false);
                        }
                        else
                        {
                            sick_scansegment_xd::CompactDataParser::Parse(udp_buffer, fifo_clock::now(), segment, 0, true, false);
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

                        const size_t idx = segment.segmentIndex;
                        samples[idx].emplace_front();
                        samples[idx].resize(this->config.max_segment_buffering);
                        swapSegmentsNoIMU(samples[idx].front(), segment);
                        filled_segments |= 1 << idx;

                        if(filled_segments >= (1 << MS100_SEGMENTS_PER_FRAME) - 1)
                        {
                            // assemble and publish pc
                            sensor_msgs::msg::PointCloud2 scan;
                            

                            this->scan_pub->publish(scan);
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

            // SOPAS stop command
        }
        // sleep
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


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiscanNode>());
    rclcpp::shutdown();

    return 0;
}
