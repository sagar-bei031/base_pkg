#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"

class SerialNode : public rclcpp::Node
{
public:
    SerialNode() : Node("serial_node")
    {
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("robot/odom0", 10);
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&SerialNode::joy_callback, this, std::placeholders::_1));

        serial_port_.open("/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0", 115200);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&SerialNode::process_buffer, this));

        odom_seq_ = 0;
    }

    void joy_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
    {
        std::vector<uint8_t> data;
        uint8_t start_byte = 0xA5;
        data.push_back(start_byte);

        float lx = twist_msg->linear.x;
        float ly = twist_msg->linear.y;
        float aw = twist_msg->angular.w;

        std::memcpy(&lx, &lx, sizeof(lx));
        std::memcpy(&ly, &ly, sizeof(ly));
        std::memcpy(&aw, &aw, sizeof(aw));

        data.insert(data.end(), reinterpret_cast<const uint8_t *>(&lx), reinterpret_cast<const uint8_t *>(&lx) + sizeof(lx));
        data.insert(data.end(), reinterpret_cast<const uint8_t *>(&ly), reinterpret_cast<const uint8_t *>(&ly) + sizeof(ly));
        data.insert(data.end(), reinterpret_cast<const uint8_t *>(&aw), reinterpret_cast<const uint8_t *>(&aw) + sizeof(aw));

        uint8_t hash = calc_crc(data.data() + 1, data.size() - 1);
        data.push_back(hash);

        serial_port_.write(data.data(), data.size());
        serial_port_.reset_output_buffer();
    }

    void odom_serial_receive()
    {
        while (true)
        {
            if (serial_port_.available() >= 26)
            {
                bool start_byte_found = false;
                while (!start_byte_found)
                {
                    uint8_t byte;
                    serial_port_.read(&byte, 1);
                    if (byte == 0xA5)
                    {
                        std::vector<uint8_t> data_str(25);
                        serial_port_.read(data_str.data(), 25);
                        start_byte_found = true;
                    }
                }

                std::vector<uint8_t> data_str(25);
                serial_port_.read(data_str.data(), 25);
                uint8_t hash = calc_crc(data_str.data(), data_str.size() - 1);

                if (hash == data_str.back())
                {
                    serial_port_.reset_input_buffer();
                    float data[6];
                    std::memcpy(data, data_str.data(), 24); // data = [x, y, theta, vx, vy, omega]
                    // this->position.append(data[0], data[1], 0.0, 0.0, 0.0, data[2]);  // [x, y, z, roll, pitch, yaw]
                    // this->twist_buffer.append(data[3], data[4], 0.0, 0.0, 0.0, data[5]); // [vx, vy, vz, v, vroll, vpitch, vyaw]
                }
            }
            else
            {
                break;
            }
        }
    }

    void process_buffer()
    {
        if (!pose_buffer.empty() && !twist_buffer.empty())
        {
            auto pose_means = Eigen::Map<Eigen::VectorXf>(pose_buffer.data(), pose_buffer.size()).mean();
            auto pose_covs = Eigen::Map<Eigen::MatrixXf>(pose_buffer.data(), pose_buffer.size(), pose_buffer.size()).cov();
            auto twist_means = Eigen::Map<Eigen::VectorXf>(twist_buffer.data(), twist_buffer.size()).mean();
            auto twist_covs = Eigen::Map<Eigen::MatrixXf>(twist_buffer.data(), twist_buffer.size(), twist_buffer.size()).cov();

            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header.seq = odom_seq_;
            odom_msg.header.stamp = this->get_clock()->now();
            odom_msg.header.frame_id = "odom";
            odom_msg.child_frame_id = "base_link";

            odom_msg.pose.pose.position = geometry_msgs::msg::Point();
            odom_msg.pose.pose.position.x = pose_means[0];
            odom_msg.pose.pose.position.y = pose_means[1];
            odom_msg.pose.pose.position.z = pose_means[2];

            auto [qx, qy, qz, qw] = euler_to_quaternion(pose_means[3], pose_means[4], pose_means[5]);
            odom_msg.pose.pose.orientation.x = qx;
            odom_msg.pose.pose.orientation.y = qy;
            odom_msg.pose.pose.orientation.z = qz;
            odom_msg.pose.pose.orientation.w = qw;

            odom_msg.pose.covariance = pose_covs;

            odom_msg.twist.twist = geometry_msgs::msg::Twist();
            odom_msg.twist.twist.linear.x = twist_means[0];
            odom_msg.twist.twist.linear.y = twist_means[1];
            odom_msg.twist.twist.linear.z = twist_means[2];
            odom_msg.twist.twist.angular.x = twist_means[3];
            odom_msg.twist.twist.angular.y = twist_means[4];
            odom_msg.twist.twist.angular.z = twist_means[5];

            odom_msg.twist.covariance = twist_covs;

            odom_publisher_->publish(odom_msg);
            odom_seq_++;
            pose_buffer.clear();
            twist_buffer.clear();
        }
    }

    uint8_t calc_crc(const uint8_t *data, size_t size)
    {
        // Implement CRC calculation
        return 0;
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    serial::Serial serial_port_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t odom_seq_;
    std::vector<float> pose_buffer;
    std::vector<float> twist_buffer;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto myserial = std::make_shared<SerialNode>();
    try
    {
        rclcpp::spin(myserial);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception caught: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "Unknown exception caught" << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}
