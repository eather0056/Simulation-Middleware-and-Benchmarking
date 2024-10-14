#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
//#include "std_msgs/msg/string.hpp"

class SumNode : public rclcpp::Node {
public:
    SumNode() : Node("sum_node") {
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "input_topic", 10, std::bind(&SumNode::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data);
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SumNode>());
    rclcpp::shutdown();
    return 0;
}
