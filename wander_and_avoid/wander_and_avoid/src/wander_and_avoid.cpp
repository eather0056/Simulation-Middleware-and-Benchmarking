#include <chrono>
#include <memory>
#include <cmath>
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

class WanderAndAvoid : public rclcpp::Node
{
public:
    WanderAndAvoid()
    : Node("wander_and_avoid"), obstacle_detected_(false), turning_(false), min_distance_(0.0), turn_direction_(1.0)
    {
        vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/robot1/cmd_vel", 10);
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/robot1/scan", 10, std::bind(&WanderAndAvoid::scan_callback, this, std::placeholders::_1));
        timer_ = create_wall_timer(100ms, std::bind(&WanderAndAvoid::wander, this));
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        obstacle_detected_ = false;
        min_distance_ = msg->range_max;

        int num_ranges = msg->ranges.size();
        float angle_increment = 360.0 / num_ranges;
        int end_index_1 = 30 / angle_increment;
        int start_index_2 = num_ranges - end_index_1;

        auto check_obstacle = [&](int start, int end) {
            for (int i = start; i <= end; ++i) {
                float range = msg->ranges[i];
                if (std::isfinite(range) && range < 0.4) {
                    obstacle_detected_ = true;
                    min_distance_ = range;
                    return true;
                }
            }
            return false;
        };

        check_obstacle(0, end_index_1) || check_obstacle(start_index_2, num_ranges - 1);
    }

    void wander()
    {
        auto cmd_msg = geometry_msgs::msg::Twist();

        if (obstacle_detected_) {
            if (!turning_) {
                turn_direction_ = (std::rand() % 2 == 0) ? 1.0 : -1.0;
                turning_ = true;
            }
            cmd_msg.angular.z = 0.5 * turn_direction_;
            cmd_msg.linear.x = 0.0;
        } else {
            turning_ = false;
            cmd_msg.linear.x = 0.2;
            cmd_msg.angular.z = (std::rand() % 2 == 0 ? 0.2 : -0.2);
        }

        vel_pub_->publish(cmd_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool obstacle_detected_;
    bool turning_;
    float min_distance_;
    float turn_direction_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WanderAndAvoid>());
    rclcpp::shutdown();
    return 0;
}
