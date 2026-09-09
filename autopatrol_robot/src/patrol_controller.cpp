#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>

#include "autopatrol_robot/srv/speech_text.hpp"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

struct Waypoint {
  std::string name;
  double x{};
  double y{};
  double yaw{};
  std::string text;
};

class PatrolController final : public rclcpp::Node {
 public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using SpeechText = autopatrol_robot::srv::SpeechText;

  PatrolController()
  : Node("patrol_controller") {
    waypoints_file_ = declare_parameter<std::string>("waypoints_file", "");
    image_topic_ = declare_parameter<std::string>("image_topic", "/camera_sensor/image_raw");
    image_save_dir_ = declare_parameter<std::string>("image_save_dir", "./autopatrol_images");
    speech_service_ = declare_parameter<std::string>("speech_service", "/speech_text");
    navigate_action_ = declare_parameter<std::string>("navigate_action", "navigate_to_pose");
    loop_enabled_ = declare_parameter<bool>("loop_enabled", true);
    goal_timeout_sec_ = declare_parameter<double>("goal_timeout", 180.0);
    image_wait_timeout_sec_ = declare_parameter<double>("image_wait_timeout", 2.0);
    speech_timeout_sec_ = declare_parameter<double>("speech_timeout", 15.0);

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PatrolController::onImage, this, std::placeholders::_1));
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, navigate_action_);
    speech_client_ = create_client<SpeechText>(speech_service_);
    timer_ = create_wall_timer(200ms, std::bind(&PatrolController::tick, this));

    if (!loadWaypoints()) {
      RCLCPP_ERROR(get_logger(), "巡检配置加载失败，节点不会发送导航目标");
      stopped_ = true;
    }
  }

 private:
  enum class State { 
    WaitingNav, 
    Navigating, 
    WaitingImage, 
    WaitingSpeech, 
    Stopped 
  };

  bool loadWaypoints() {
    if (waypoints_file_.empty()) {
      RCLCPP_ERROR(get_logger(), "参数 waypoints_file 不能为空");
      return false;
    }
    try {
      const auto root = YAML::LoadFile(waypoints_file_);
      const auto nodes = root["waypoints"];
      if (!nodes || !nodes.IsSequence() || nodes.size() == 0) {
        RCLCPP_ERROR(get_logger(), "waypoints 必须是非空 YAML 列表");
        return false;
      }
      std::set<std::string> names;
      for (const auto & node : nodes) {
        if (!node["name"] || !node["x"] || !node["y"] || !node["yaw"]) {
          RCLCPP_ERROR(get_logger(), "每个巡检点必须包含 name/x/y/yaw");
          return false;
        }
        Waypoint point;
        point.name = node["name"].as<std::string>();
        point.x = node["x"].as<double>();
        point.y = node["y"].as<double>();
        point.yaw = node["yaw"].as<double>();
        point.text = node["text"] ? node["text"].as<std::string>() : point.name;
        if (point.name.empty() || !std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.yaw) || !names.insert(point.name).second) {
          RCLCPP_ERROR(get_logger(), "巡检点名称必须唯一，坐标和 yaw 必须是有限数值");
          return false;
        }
        waypoints_.push_back(std::move(point));
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "读取 YAML 失败: %s", ex.what());
      return false;
    }
    RCLCPP_INFO(get_logger(), "已加载 %zu 个巡检点", waypoints_.size());
    return true;
  }

  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    try {
      const auto cv_image = cv_bridge::toCvCopy(msg, "bgr8");
      std::lock_guard<std::mutex> lock(image_mutex_);
      latest_image_ = cv_image->image.clone();
      latest_image_time_ = std::chrono::steady_clock::now();
    } catch (const cv_bridge::Exception & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "图像转换失败: %s", ex.what());
    }
  }

  void tick() {
    if (stopped_ || state_ == State::Stopped || waypoints_.empty()) return;
    if (state_ == State::WaitingNav) {
      if (nav_client_->action_server_is_ready()) sendCurrentGoal();
      return;
    }
    if (state_ == State::Navigating &&
        std::chrono::steady_clock::now() - nav_started_ > std::chrono::duration<double>(goal_timeout_sec_)) {
      RCLCPP_ERROR(get_logger(), "巡检点 %s 导航超时，跳过", waypoints_[index_].name.c_str());
      const auto old_handle = goal_handle_;
      ++goal_token_;
      goal_handle_.reset();
      if (old_handle) nav_client_->async_cancel_goal(old_handle);
      finishWaypoint();
      return;
    }
    if (state_ == State::WaitingImage) {
      bool have_image = false;
      {
        std::lock_guard<std::mutex> lock(image_mutex_);
        have_image = !latest_image_.empty();
      }
      if (have_image) {
        saveLatestImage(waypoints_[index_].name);
        beginSpeech();
      } else if (std::chrono::steady_clock::now() - image_started_ >
                 std::chrono::duration<double>(image_wait_timeout_sec_)) {
        RCLCPP_WARN(get_logger(), "巡检点 [%s] 等待图像超时", waypoints_[index_].name.c_str());
        beginSpeech();
      }
      return;
    }
    if (state_ == State::WaitingSpeech &&
        std::chrono::steady_clock::now() - speech_started_ > std::chrono::duration<double>(speech_timeout_sec_)) {
      RCLCPP_WARN(get_logger(), "语音服务超时，继续巡检");
      finishWaypoint();
    }
  }

  void sendCurrentGoal() {
    const auto point = waypoints_[index_];
    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = point.x;
    goal.pose.pose.position.y = point.y;
    goal.pose.pose.orientation.z = std::sin(point.yaw / 2.0);
    goal.pose.pose.orientation.w = std::cos(point.yaw / 2.0);
    const auto token = ++goal_token_;
    state_ = State::Navigating;
    nav_started_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "前往巡检点 [%s] (%.2f, %.2f)", point.name.c_str(), point.x, point.y);

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback = [this, token](GoalHandle::SharedPtr handle) {
      if (token != goal_token_) return;
      if (!handle) {
        RCLCPP_ERROR(get_logger(), "导航目标被拒绝，跳过当前点");
        finishWaypoint();
        return;
      }
      goal_handle_ = handle;
    };
    options.result_callback = [this, token](const GoalHandle::WrappedResult & result) {
      if (token != goal_token_ || state_ != State::Navigating) return;
      goal_handle_.reset();
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        handleArrival();
      } else {
        RCLCPP_ERROR(get_logger(), "巡检点 [%s] 导航失败，跳过", waypoints_[index_].name.c_str());
        finishWaypoint();
      }
    };
    nav_client_->async_send_goal(goal, options);
  }

  void handleArrival() {
    if (!saveLatestImage(waypoints_[index_].name)) {
      state_ = State::WaitingImage;
      image_started_ = std::chrono::steady_clock::now();
      return;
    }
    beginSpeech();
  }

  void beginSpeech() {
    const auto & point = waypoints_[index_];
    if (point.text.empty() || !speech_client_->service_is_ready()) {
      if (!point.text.empty()) RCLCPP_WARN(get_logger(), "语音服务不可用，跳过播报");
      finishWaypoint();
      return;
    }
    auto request = std::make_shared<SpeechText::Request>();
    request->text = point.text;
    const auto token = goal_token_;
    state_ = State::WaitingSpeech;
    speech_started_ = std::chrono::steady_clock::now();
    speech_client_->async_send_request(request, [this, token](
      std::shared_future<SpeechText::Response::SharedPtr> future) {
      if (token != goal_token_ || state_ != State::WaitingSpeech) return;
      const auto response = future.get();
      if (!response->success) RCLCPP_WARN(get_logger(), "语音播报失败: %s", response->message.c_str());
      finishWaypoint();
    });
  }

  bool saveLatestImage(const std::string & name) {
    cv::Mat image;
    {
      std::lock_guard<std::mutex> lock(image_mutex_);
      if (latest_image_.empty()) {
        RCLCPP_WARN(get_logger(), "巡检点 [%s] 没有可用图像", name.c_str());
        return false;
      }
      image = latest_image_.clone();
    }
    std::error_code ec;
    std::filesystem::create_directories(image_save_dir_, ec);
    if (ec) {
      RCLCPP_ERROR(get_logger(), "创建图像目录失败: %s", ec.message().c_str());
      return false;
    }
    const auto time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm{};
    localtime_r(&time, &tm);
    std::ostringstream filename;
    filename << image_save_dir_ << "/" << name << "_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".jpg";
    if (!cv::imwrite(filename.str(), image)) {
      RCLCPP_ERROR(get_logger(), "保存图像失败: %s", filename.str().c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "已保存巡检图像: %s", filename.str().c_str());
    return true;
  }

  void finishWaypoint() {
    if (state_ == State::Stopped) return;
    ++index_;
    if (index_ >= waypoints_.size()) {
      if (!loop_enabled_) {
        state_ = State::Stopped;
        RCLCPP_INFO(get_logger(), "巡检完成");
        return;
      }
      index_ = 0;
      RCLCPP_INFO(get_logger(), "一轮巡检完成，重新开始");
    }
    state_ = State::WaitingNav;
  }

  std::vector<Waypoint> waypoints_;
  std::string waypoints_file_, image_topic_, image_save_dir_, speech_service_, navigate_action_;
  bool loop_enabled_{true}, stopped_{false};
  double goal_timeout_sec_{180.0}, image_wait_timeout_sec_{2.0}, speech_timeout_sec_{15.0};
  size_t index_{0};
  State state_{State::WaitingNav};
  uint64_t goal_token_{0};
  GoalHandle::SharedPtr goal_handle_;
  std::chrono::steady_clock::time_point nav_started_, speech_started_;
  std::chrono::steady_clock::time_point image_started_;
  cv::Mat latest_image_;
  std::chrono::steady_clock::time_point latest_image_time_;
  std::mutex image_mutex_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Client<SpeechText>::SharedPtr speech_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PatrolController>());
  rclcpp::shutdown();
  return 0;
}
