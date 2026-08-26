#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SubNovelNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
private:

public:
    SubNovelNode(std::string name) : Node(name) {
        auto qos = rclcpp::QoS(10);
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "novel_content", 
            qos, 
            [this](const std_msgs::msg::String::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "小说内容：%s", msg->data.c_str());
            }
        );
    }
    ~SubNovelNode() {

    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SubNovelNode>("sub_novel_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
