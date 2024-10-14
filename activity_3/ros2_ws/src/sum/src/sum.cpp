#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class SumNode : public rclcpp::Node {
public:
    SumNode() : Node("sum_node"), sum_(0) {
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "input_topic", 10, std::bind(&SumNode::topic_callback, this, std::placeholders::_1));

        // Publisher can be added in Version 5
        // publisher_ = this->create_publisher<std_msgs::msg::Int32>("output_topic", 10);
    }

private:
    void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        sum_ += msg->data;
        RCLCPP_INFO(this->get_logger(), "Sum: '%d'", sum_);
        
        // For Version 5, to publish sum
        // auto message = std_msgs::msg::Int32();
        // message.data = sum_;
        // publisher_->publish(message);
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    // Add a publisher for Version 5
    // rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    int sum_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SumNode>());
    rclcpp::shutdown();
    return 0;
}
