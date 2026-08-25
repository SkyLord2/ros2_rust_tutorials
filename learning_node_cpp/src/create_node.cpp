#include <rclcpp/rclcpp.hpp>

class  NormalNode : public rclcpp::Node
{
private:
    /* data */
public:
    NormalNode(std::string name) : Node(name) {
        RCLCPP_INFO(this->get_logger(), "ROS2测试，创建节点");
    }
    ~ NormalNode() {

    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NormalNode>("normal_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}