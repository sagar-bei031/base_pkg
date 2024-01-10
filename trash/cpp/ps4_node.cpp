#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>

constexpr double MAX_VELOCITY = 1.0;
constexpr double MAX_OMEGA = 1.5;

double map_value(double value, double min_value, double max_value, double new_min, double new_max) {
    return ((value - min_value) * (new_max - new_min)) / (max_value - min_value) + new_min;
}

class PS4Node : public rclcpp::Node {
public:
    PS4Node() : Node("ps4_node") {
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&PS4Node::joy_callback, this, std::placeholders::_1));
        is_emergency_brake_ = false;
        last_published_time_ = std::chrono::system_clock::now();
        this->get_logger()->info("ps4_node is running...");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        auto current_time = std::chrono::system_clock::now();
        auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_published_time_).count() / 1000.0;

        if (dt > 0.02) {
            double speed_factor = (msg->buttons[5] == 1) ? 0.2 : 1.0;

            double vx = map_value(msg->axes[0], -1.0, 1.0, MAX_VELOCITY, -MAX_VELOCITY) * speed_factor;
            double vy = map_value(msg->axes[1], -1.0, 1.0, -MAX_VELOCITY, MAX_VELOCITY) * speed_factor;
            double w = map_value(msg->axes[2] - msg->axes[5], -2.0, 2.0, MAX_OMEGA, -MAX_OMEGA) * speed_factor;

            if ((msg->axes[6] != 0) || (msg->axes[7] != 0)) {
                vy = msg->axes[7] * MAX_VELOCITY * speed_factor;
                vx = -msg->axes[6] * MAX_VELOCITY * speed_factor;
            }

            if (msg->buttons[10]) {
                is_emergency_brake_ = true;
            }

            if (msg->buttons[4] && msg->buttons[5] && msg->buttons[10]) {
                is_emergency_brake_ = false;
            }

            if (is_emergency_brake_) {
                vx = vy = w = 0.0;
            }

            set_speed(vx, vy, w);
            last_published_time_ = current_time;
        }
    }

    void set_speed(double vx, double vy, double w) {
        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = vx;
        twist_msg->linear.y = vy;
        twist_msg->angular.z = w;
        cmd_publisher_->publish(std::move(twist_msg));
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    bool is_emergency_brake_;
    std::chrono::time_point<std::chrono::system_clock> last_published_time_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto ps4 = std::make_shared<PS4Node>();
    rclcpp::spin(ps4);
    rclcpp::shutdown();
    return 0;
}
