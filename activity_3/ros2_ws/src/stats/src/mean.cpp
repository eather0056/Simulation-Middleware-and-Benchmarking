#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"

class MeanNode : public rclcpp::Node {
public:
    MeanNode() : Node("mean_node"), sum_(0), count_(0) {
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "input_topic", 10, std::bind(&MeanNode::topic_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<std_msgs::msg::Float64>("mean", 10);
    }

private:
    void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        sum_ += msg->data;
        count_++;

        double mean = static_cast<double>(sum_) / count_;
        RCLCPP_INFO(this->get_logger(), "Mean: %f", mean);

        auto message = std_msgs::msg::Float64();
        message.data = mean;
        publisher_->publish(message);
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    int sum_;
    int count_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MeanNode>());
    rclcpp::shutdown();
    return 0;
}
