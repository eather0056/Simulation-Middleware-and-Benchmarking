#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class SumNode : public rclcpp::Node {
public:
    SumNode() : Node("sum_node"), sum_(0) {
        // Subscription to "input_topic"
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "input_topic", 10, std::bind(&SumNode::topic_callback, this, std::placeholders::_1));

        // Publisher for "output_topic"
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("output_topic", 10);
    }

private:
    void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        // Add the incoming value to the sum
        sum_ += msg->data;

        // Log the current sum
        RCLCPP_INFO(this->get_logger(), "Current sum: '%d'", sum_);

        // Create a message to publish the sum
        auto message = std_msgs::msg::Int32();
        message.data = sum_;

        // Publish the sum to the output topic
        publisher_->publish(message);
    }

    // Subscription to receive input data
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;

    // Publisher to publish the sum result
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;

    // Variable to store the accumulated sum
    int sum_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SumNode>());
    rclcpp::shutdown();
    return 0;
}
