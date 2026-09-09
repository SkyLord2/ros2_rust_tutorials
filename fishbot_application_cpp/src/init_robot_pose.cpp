#include <chrono>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class RobotPoseInitializer final : public rclcpp::Node {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  RobotPoseInitializer()
  : Node(
      "robot_pose_initializer",
      rclcpp::NodeOptions().append_parameter_override("use_sim_time", true)) {
    initial_x_ = declare_parameter<double>("initial_x", 0.0);
    initial_y_ = declare_parameter<double>("initial_y", 0.0);
    initial_yaw_ = declare_parameter<double>("initial_yaw", 0.0);
    max_publish_count_ = declare_parameter<int>("initial_pose_publish_count", 20);

    initial_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", rclcpp::QoS(10).reliable());
    amcl_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", rclcpp::QoS(10),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr) {
        if (!localized_) {
          localized_ = true;
          publish_timer_->cancel();
          RCLCPP_INFO(get_logger(), "已收到 AMCL 定位结果，停止重复发布初始位姿");
        }
      });
    nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // AMCL can start after the publisher is created. Repeating the pose avoids
    // losing the one-shot message during DDS discovery or lifecycle startup.
    publish_timer_ = create_wall_timer(1s, std::bind(&RobotPoseInitializer::publishIfReady, this));
    RCLCPP_INFO(get_logger(), "初始位姿节点已启动，等待 Nav2 和 AMCL 就绪...");
  }

private:
  void publishIfReady() {
    if (!nav_to_pose_client_->action_server_is_ready()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "等待 Nav2 导航服务启动...");
      return;
    }
    if (initial_pose_pub_->get_subscription_count() == 0) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "等待 AMCL 订阅 /initialpose...");
      return;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = now();
    pose.pose.pose.position.x = initial_x_;
    pose.pose.pose.position.y = initial_y_;
    pose.pose.pose.orientation.z = std::sin(initial_yaw_ / 2.0);
    pose.pose.pose.orientation.w = std::cos(initial_yaw_ / 2.0);
    pose.pose.covariance[0] = 0.25;
    pose.pose.covariance[7] = 0.25;
    pose.pose.covariance[35] = 0.06;
    initial_pose_pub_->publish(pose);
    ++publish_count_;
    RCLCPP_INFO(
      get_logger(), "已发布初始位姿 (%d/%d): x=%.2f y=%.2f yaw=%.2f",
      publish_count_, max_publish_count_, initial_x_, initial_y_, initial_yaw_);

    if (publish_count_ >= max_publish_count_) {
      publish_timer_->cancel();
      RCLCPP_INFO(get_logger(), "初始位姿重复发布完成");
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  double initial_x_{0.0};
  double initial_y_{0.0};
  double initial_yaw_{0.0};
  int max_publish_count_{20};
  int publish_count_{0};
  bool localized_{false};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotPoseInitializer>());
  rclcpp::shutdown();
  return 0;
}
