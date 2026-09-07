#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;

class RobotPoseInitializer : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;

    RobotPoseInitializer() : Node("robot_pose_initializer") {
        // 创建初始位姿发布者，对应 Python 中 setInitialPose 的底层发布话题
        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10);

        // 创建 Nav2 导航 Action 客户端，用于判断 Nav2 导航栈是否启动就绪
        nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");

        RCLCPP_INFO(this->get_logger(), "节点已启动，准备初始化机器人位姿...");
    }

    // 设置并发布机器人初始位姿
    void setInitialPose(double x, double y) {
        auto init_pose = geometry_msgs::msg::PoseWithCovarianceStamped();
        
        // 关键配置：指定参考坐标系为 map
        init_pose.header.frame_id = "map";
        init_pose.header.stamp = this->now();

        // 设置平面二维位置
        init_pose.pose.pose.position.x = x;
        init_pose.pose.pose.position.y = y;
        init_pose.pose.pose.position.z = 0.0;

        // 设置朝向四元数（注意：单位四元数标准默认值为 w = 1.0，避免四元数模长为 0 引发数学错误）
        init_pose.pose.pose.orientation.x = 0.0;
        init_pose.pose.pose.orientation.y = 0.0;
        init_pose.pose.pose.orientation.z = 0.0;
        init_pose.pose.pose.orientation.w = 1.0;

        // 设置初始位姿协方差矩阵（默认轻微不确定度）
        init_pose.pose.covariance[0] = 0.25;  // x 方差
        init_pose.pose.covariance[7] = 0.25;  // y 方差
        init_pose.pose.covariance[35] = 0.06; // yaw 偏航角方差

        // 发布初始位姿给定位模块（如 AMCL）
        initial_pose_pub_->publish(init_pose);
        RCLCPP_INFO(this->get_logger(), "已发布初始位姿: x=%.2f, y=%.2f", x, y);
    }

    // 等待导航可用（等效于 Python 的 navigator.waitUntilNav2Active()）
    bool waitUntilNav2Active(std::chrono::seconds timeout = 30s) {
        RCLCPP_INFO(this->get_logger(), "等待 Nav2 导航服务启动...");
        
        // 循环等待 navigate_to_pose 动作服务器就绪
        auto start_time = std::chrono::steady_clock::now();
        while (!nav_to_pose_client_->wait_for_action_server(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "系统被中断");
                return false;
            }
            if (std::chrono::steady_clock::now() - start_time > timeout) {
                RCLCPP_ERROR(this->get_logger(), "等待 Nav2 超时！");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Nav2 尚未就绪，继续等待...");
        }

        RCLCPP_INFO(this->get_logger(), "Nav2 导航栈已处于激活状态！");
        return true;
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RobotPoseInitializer>();

    // 1. 严格先等待 Nav2 核心服务激活就绪
    if (node->waitUntilNav2Active()) {
        // 2. 确认就绪后，再发布初始位姿 (0.0, 0.0)
        node->setInitialPose(0.0, 0.0);
    }
    // 3. 持续运行节点
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}