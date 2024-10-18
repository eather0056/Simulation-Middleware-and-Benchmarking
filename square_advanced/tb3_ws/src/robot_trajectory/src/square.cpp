#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class SquareTrajectory : public rclcpp::Node
{
public:
    SquareTrajectory() : Node("square"), count_(0)
    {
        // Create a publisher to the /cmd_vel topic
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        // Set up a timer to control the robot's movement
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&SquareTrajectory::move_robot, this));
    }

private:
    void move_robot()
    {
        // Create Twist message for robot velocity
        geometry_msgs::msg::Twist cmd_vel_msg;

        // Move straight for 1 meter (10 iterations with 0.1m/s)
        if (count_ < 10)
        {
            cmd_vel_msg.linear.x = 0.1; // Move forward with 0.1 m/s
            cmd_vel_msg.angular.z = 0.0; // No rotation
        }
        // Rotate 90 degrees (20 iterations with 9 degrees/s)
        else if (count_ < 30)
        {
            cmd_vel_msg.linear.x = 0.0; // No forward movement
            cmd_vel_msg.angular.z = 0.157; // Rotate with 9 degrees/s (0.157 rad/s)
        }
        // Stop after completing one side
        else if (count_ == 30)
        {
            cmd_vel_msg.linear.x = 0.0;
            cmd_vel_msg.angular.z = 0.0;
        }

        // Publish the command
        publisher_->publish(cmd_vel_msg);

        // Reset the counter after a full square loop
        count_++;
        if (count_ > 40)
        {
            count_ = 0;
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SquareTrajectory>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

