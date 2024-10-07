#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>

void topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "I heard: '%s'", msg->data.c_str());
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("subscriber");

    auto subscription = node->create_subscription<std_msgs::msg::String>(
        "topic", 10, topic_callback);

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
