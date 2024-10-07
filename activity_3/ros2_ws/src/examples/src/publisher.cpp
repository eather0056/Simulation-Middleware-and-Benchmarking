#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp" 

using namespace std::chrono_literals;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = rclcpp::Node::make_shared("publisher");
    
    auto publisher = node->create_publisher<std_msgs::msg::String>("topic", 10);  // Fix: Correct the topic string and template

    std_msgs::msg::String message;
    auto publish_count = 0;

    rclcpp::WallRate loop_rate(500ms); 

    while (rclcpp::ok())
    {
        message.data = "Hello, World! " + std::to_string(publish_count++);
        
        publisher->publish(message);
        
        loop_rate.sleep(); 

    }

    rclcpp::shutdown();
    return 0;
}
