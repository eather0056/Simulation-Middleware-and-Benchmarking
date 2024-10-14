#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"
#include <vector>
#include <algorithm>

class MedianNode : public rclcpp::Node {
public:
    MedianNode() : Node("median_node") {
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "input_topic", 10, std::bind(&MedianNode::topic_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<std_msgs::msg::Float64>("median", 10);
    }

private:
    void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        numbers_.push_back(msg->data);

        std::vector<int> sorted_numbers = numbers_;
        std::sort(sorted_numbers.begin(), sorted_numbers.end());

        double median;
        size_t size = sorted_numbers.size();
        if (size % 2 == 0) {
            median = (sorted_numbers[size / 2 - 1] + sorted_numbers[size / 2]) / 2.0;
        } else {
            median = sorted_numbers[size / 2];
        }

        RCLCPP_INFO(this->get_logger(), "Median: %f", median);

        auto message = std_msgs::msg::Float64();
        message.data = median;
        publisher_->publish(message);
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    std::vector<int> numbers_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MedianNode>());
    rclcpp::shutdown();
    return 0;
}
