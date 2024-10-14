#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <map>
#include <vector>

class ModeNode : public rclcpp::Node {
public:
    ModeNode() : Node("mode_node") {
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "input_topic", 10, std::bind(&ModeNode::topic_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<std_msgs::msg::Int32>("mode", 10);
    }

private:
    void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        number_counts_[msg->data]++;

        int mode = 0;
        int max_count = 0;

        for (const auto& [number, count] : number_counts_) {
            if (count > max_count) {
                mode = number;
                max_count = count;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Mode: %d", mode);

        auto message = std_msgs::msg::Int32();
        message.data = mode;
        publisher_->publish(message);
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    std::map<int, int> number_counts_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModeNode>());
    rclcpp::shutdown();
    return 0;
}
