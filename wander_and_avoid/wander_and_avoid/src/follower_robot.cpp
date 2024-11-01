#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>
#include <cstdlib>
#include <cmath>

using namespace std::chrono_literals;

class FollowerRobot : public rclcpp::Node
{
public:
    FollowerRobot() : Node("follower_robot"), obstacle_detected_(false), turning_(false), turn_direction_(1.0)
    {
        // Initialize publisher, subscriber, and tf listener
        vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/robot2/cmd_vel", 10);
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/robot2/scan", 10, std::bind(&FollowerRobot::scan_callback, this, std::placeholders::_1));
        timer_ = create_wall_timer(100ms, std::bind(&FollowerRobot::follow, this));

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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
                if (std::isfinite(range) && range < 0.5) { // Threshold distance for obstacle avoidance
                    obstacle_detected_ = true;
                    min_distance_ = range;
                    return true;
                }
            }
            return false;
        };

        check_obstacle(0, end_index_1) || check_obstacle(start_index_2, num_ranges - 1);
    }

    void follow()
    {
        geometry_msgs::msg::Twist cmd_msg;

        if (obstacle_detected_) {
            // Obstacle avoidance logic
            if (!turning_) {
                turn_direction_ = (std::rand() % 2 == 0) ? 1.0 : -1.0;
                turning_ = true;
            }
            cmd_msg.angular.z = 0.8 * turn_direction_; // Continue turning to avoid obstacle
            cmd_msg.linear.x = 0.0;
        } else {
            turning_ = false;

            // Follow robot1 using tf
            try {
                if (!tf_buffer_->canTransform("robot2/base_footprint", "robot1/base_footprint", tf2::TimePointZero)) {
                    return; // Skip this cycle if the transform is not yet available
                }

                auto transform = tf_buffer_->lookupTransform("robot2/base_footprint", "robot1/base_footprint", tf2::TimePointZero);

                // Calculate distance and angle to robot1
                double distance_to_leader = std::hypot(transform.transform.translation.x, transform.transform.translation.y);
                double angle_to_leader = std::atan2(transform.transform.translation.y, transform.transform.translation.x);

                // Adjust angular velocity if misaligned
                if (std::abs(angle_to_leader) > 0.1) {
                    cmd_msg.angular.z = 0.8 * angle_to_leader;
                }

                // Move forward if within following distance range
                if (distance_to_leader > 0.5) {
                    cmd_msg.linear.x = std::min(0.3, 0.1 * distance_to_leader);
                } else {
                    cmd_msg.linear.x = 0.0; // Stop if close enough to robot1
                }

            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Could not transform robot1 to robot2: %s", ex.what());
            }
        }

        vel_pub_->publish(cmd_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    bool obstacle_detected_;
    bool turning_;
    float min_distance_;
    float turn_direction_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowerRobot>());
    rclcpp::shutdown();
    return 0;
}
