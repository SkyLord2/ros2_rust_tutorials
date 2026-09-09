#include <chrono>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/srv/set_initial_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class RobotPoseInitializer final : public rclcpp::Node {
public:
  using SetInitialPose = nav2_msgs::srv::SetInitialPose;

  RobotPoseInitializer()
  : Node("robot_pose_initializer"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {
    initial_x_ = declare_parameter<double>("initial_x", 0.0);
    initial_y_ = declare_parameter<double>("initial_y", 0.0);
    initial_yaw_ = declare_parameter<double>("initial_yaw", 0.0);
    initial_frame_ = declare_parameter<std::string>("initial_frame", "map");
    initial_pose_topic_ = declare_parameter<std::string>("initial_pose_topic", "/initialpose");
    amcl_service_ = declare_parameter<std::string>("amcl_service", "/amcl/set_initial_pose");
    max_publish_count_ = declare_parameter<int>("initial_pose_publish_count", 20);

    initial_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initial_pose_topic_, rclcpp::QoS(1).reliable().transient_local());
    amcl_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", rclcpp::QoS(10),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr) {
        if (!localized_) {
          localized_ = true;
          publish_timer_->cancel();
          RCLCPP_INFO(get_logger(), "已收到 AMCL 定位结果，停止重复发布初始位姿");
        }
      });
    initial_pose_client_ = create_client<SetInitialPose>(amcl_service_);

    publish_timer_ = create_wall_timer(1s, std::bind(&RobotPoseInitializer::publishIfReady, this));
    RCLCPP_INFO(
      get_logger(), "初始位姿节点已启动: (%.2f, %.2f, %.2f), frame=%s, service=%s",
      initial_x_, initial_y_, initial_yaw_, initial_frame_.c_str(), amcl_service_.c_str());
  }

private:
  void publishIfReady() {
    if (localized_) {
      return;
    }
    if (now().nanoseconds() == 0) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "等待 /clock 时间有效...");
      return;
    }
    if (!tf_buffer_.canTransform("odom", "base_footprint", tf2::TimePointZero)) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "等待里程计变换 odom -> base_footprint...");
      return;
    }

    const auto pose = makePose();
    if (!service_request_sent_ && initial_pose_client_->service_is_ready()) {
      auto request = std::make_shared<SetInitialPose::Request>();
      request->pose = pose;
      service_request_sent_ = true;
      initial_pose_client_->async_send_request(request, [this](
        rclcpp::Client<SetInitialPose>::SharedFuture future) {
          try {
            future.get();
            RCLCPP_INFO(get_logger(), "AMCL 已接受初始位姿");
          } catch (const std::exception & ex) {
            service_request_sent_ = false;
            RCLCPP_WARN(get_logger(), "AMCL 初始位姿服务调用失败，将重试: %s", ex.what());
          }
        });
      RCLCPP_INFO(get_logger(), "已通过 %s 请求设置初始位姿", amcl_service_.c_str());
    } else if (!service_request_sent_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "等待 AMCL 服务 %s，当前 /initialpose 订阅者=%zu",
        amcl_service_.c_str(), initial_pose_pub_->get_subscription_count());
    }

    // Keep publishing as a compatibility fallback for AMCL versions without
    // the set_initial_pose service. Transient-local QoS also survives late discovery.
    if (initial_pose_pub_->get_subscription_count() > 0 && publish_count_ < max_publish_count_) {
      initial_pose_pub_->publish(pose);
    }
    ++publish_count_;
    RCLCPP_INFO(
      get_logger(), "发布初始位姿 (%d/%d): x=%.2f y=%.2f yaw=%.2f",
      publish_count_, max_publish_count_, initial_x_, initial_y_, initial_yaw_);

    if (publish_count_ >= max_publish_count_ && service_request_sent_) {
      publish_timer_->cancel();
      RCLCPP_INFO(get_logger(), "初始位姿重复发布完成");
    }
  }

  geometry_msgs::msg::PoseWithCovarianceStamped makePose() const {
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.frame_id = initial_frame_;
    pose.header.stamp = now();
    pose.pose.pose.position.x = initial_x_;
    pose.pose.pose.position.y = initial_y_;
    pose.pose.pose.orientation.z = std::sin(initial_yaw_ / 2.0);
    pose.pose.pose.orientation.w = std::cos(initial_yaw_ / 2.0);
    pose.pose.covariance[0] = 0.25;
    pose.pose.covariance[7] = 0.25;
    pose.pose.covariance[35] = 0.06;
    return pose;
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
  rclcpp::Client<SetInitialPose>::SharedPtr initial_pose_client_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  double initial_x_{0.0};
  double initial_y_{0.0};
  double initial_yaw_{0.0};
  std::string initial_frame_;
  std::string initial_pose_topic_;
  std::string amcl_service_;
  int max_publish_count_{20};
  int publish_count_{0};
  bool service_request_sent_{false};
  bool localized_{false};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotPoseInitializer>());
  rclcpp::shutdown();
  return 0;
}
