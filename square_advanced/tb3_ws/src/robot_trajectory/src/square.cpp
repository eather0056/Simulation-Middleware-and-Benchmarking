#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

class SquareTrajectory : public rclcpp::Node
{
public:
    SquareTrajectory() : Node("square_trajectory"), count_(0), state_(0)
    {
        // Declare two ROS parameters for linear and angular speed
        this->declare_parameter("linear_speed", 0.1);  // Default linear speed: 0.1 m/s
        this->declare_parameter("angular_speed", M_PI / 20);  // Default angular speed: π/20 rad/s
        this->declare_parameter("distance", 1.0);  // Default disteance is 1.0 m

        // Retrieve the parameter values
        this->get_parameter("linear_speed", linear_speed_);
        this->get_parameter("angular_speed", angular_speed_);
        this->get_parameter("distance", distance_);

        // Calculate the number of iterations required for a 1m move (at 0.1m/s) and a 90-degree turn
        linear_iterations_ = distance_ / (0.1 * linear_speed_);  // Assuming loop rate of 10 Hz (0.1s)
        angular_iterations_ = (M_PI / 2) / (0.1 * angular_speed_);  // 90-degree turn

        // Publisher for velocity commands
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        // Timer to control movement
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&SquareTrajectory::move_turtle, this));
    }

private:
    void move_turtle()
    {
        geometry_msgs::msg::Twist cmd_vel_msg;

        if (state_ == 0)  // Moving forward
        {
            if (count_ < linear_iterations_)  // Keep moving forward
            {
                cmd_vel_msg.linear.x = linear_speed_;
                cmd_vel_msg.angular.z = 0.0;
            }
            else  // Time to turn
            {
                state_ = 1;  // Switch to turning state
                count_ = 0;  // Reset the counter for turning
            }
        }
        else if (state_ == 1)  // Turning 90 degrees
        {
            if (count_ < angular_iterations_)  // Keep rotating
            {
                cmd_vel_msg.linear.x = 0.0;
                cmd_vel_msg.angular.z = angular_speed_;
            }
            else  // Completed the turn
            {
                state_ = 0;  // Switch back to moving forward
                count_ = 0;  // Reset the counter for moving forward
            }
        }

        publisher_->publish(cmd_vel_msg);
        count_++;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double linear_speed_;
    double angular_speed_;
    double distance_;
    int count_;
    int state_;  // 0: Moving forward, 1: Turning
    double linear_iterations_;  // Number of iterations for moving 1 meter
    double angular_iterations_;  // Number of iterations for turning 90 degrees
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SquareTrajectory>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
