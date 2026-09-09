#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <limits>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

#include "autopatrol_robot/srv/speech_text.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

/// @brief 巡检点
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
    : Node("patrol_controller"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {
        waypoints_file_ = declare_parameter<std::string>("waypoints_file", "");
        image_topic_ = declare_parameter<std::string>("image_topic", "/camera_sensor/image_raw");
        image_save_dir_ = declare_parameter<std::string>("image_save_dir", "./autopatrol_images");
        speech_service_ = declare_parameter<std::string>("speech_service", "/speech_text");
        navigate_action_ = declare_parameter<std::string>("navigate_action", "navigate_to_pose");
        loop_enabled_ = declare_parameter<bool>("loop_enabled", true);
        goal_timeout_sec_ = declare_parameter<double>("goal_timeout", 180.0);
        image_wait_timeout_sec_ = declare_parameter<double>("image_wait_timeout", 2.0);
        speech_timeout_sec_ = declare_parameter<double>("speech_timeout", 60.0);
        arrival_tolerance_ = declare_parameter<double>("arrival_tolerance", 0.35);

        if (!std::isfinite(arrival_tolerance_) || arrival_tolerance_ <= 0.0) {
            RCLCPP_WARN(get_logger(), "arrival_tolerance 必须是正数，使用默认值 0.35 米");
            arrival_tolerance_ = 0.35;
        }

        // 订阅图像
        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            image_topic_, rclcpp::SensorDataQoS(),
            std::bind(&PatrolController::onImage, this, std::placeholders::_1)
        );
        // 创建目标点导航
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, navigate_action_);
        // 创建语音服务客户端
        speech_client_ = create_client<SpeechText>(speech_service_);
        // 状态机转换
        timer_ = create_wall_timer(200ms, std::bind(&PatrolController::tick, this));
        // 加载巡检点
        if (!loadWaypoints()) {
            RCLCPP_ERROR(get_logger(), "巡检配置加载失败，节点不会发送导航目标");
            stopped_ = true;
        } else {
            RCLCPP_INFO(
                get_logger(), "巡检配置: 图像话题=%s, 保存目录=%s, 语音服务=%s, 到达容差=%.2f 米",
                image_topic_.c_str(), image_save_dir_.c_str(), speech_service_.c_str(),
                arrival_tolerance_);
        }
    }

private:
    /// @brief 状态机
    enum class State { 
        WaitingNav, 
        Navigating, 
        WaitingImage, 
        WaitingSpeech, 
        Stopped 
    };

    /**
     * @brief 加载目标巡检点
     *
     * @return 加载结果
     * 
     * @note 坐标值必须为有限实数，若传入 NaN 会导致计算异常
     */
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
    /**
     * @brief 接收相机图像
    */
    void onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        constexpr std::size_t kMaxImageBytes = 100U * 1024U * 1024U;
        if (!msg || msg->height == 0 || msg->width == 0 || msg->step == 0 ||
            msg->height > 10000 || msg->width > 10000 ||
            msg->step > kMaxImageBytes / msg->height ||
            static_cast<std::size_t>(msg->step) * msg->height > msg->data.size() ||
            msg->data.size() > kMaxImageBytes) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "收到非法或过大的图像消息，已忽略: %ux%u step=%u bytes=%zu",
                msg ? msg->width : 0, msg ? msg->height : 0, msg ? msg->step : 0,
                msg ? msg->data.size() : 0);
            return;
        }
        try {
            // Do not force every camera message through cv_bridge.  Gazebo
            // may publish RGB, mono, or floating-point depth frames, and an
            // invalid encoding/stride can make cv_bridge construct an
            // invalid cv::Mat before it can report a useful error.
            const auto & encoding = msg->encoding;
            int type = -1;
            if (encoding == sensor_msgs::image_encodings::BGR8 || encoding == "8UC3") {
                type = CV_8UC3;
            } else if (encoding == sensor_msgs::image_encodings::RGB8) {
                type = CV_8UC3;
            } else if (encoding == sensor_msgs::image_encodings::BGRA8 || encoding == "8UC4") {
                type = CV_8UC4;
            } else if (encoding == sensor_msgs::image_encodings::RGBA8) {
                type = CV_8UC4;
            } else if (encoding == sensor_msgs::image_encodings::MONO8) {
                type = CV_8UC1;
            } else if (encoding == sensor_msgs::image_encodings::MONO16 || encoding == "16UC1") {
                type = CV_16UC1;
            } else if (encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
                type = CV_32FC1;
            }
            if (type < 0) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "不支持的图像编码 [%s]，已忽略", encoding.c_str());
                return;
            }
            const auto element_bytes = static_cast<std::size_t>(CV_ELEM_SIZE(type));
            const auto min_step = static_cast<std::size_t>(msg->width) * element_bytes;
            if (min_step == 0 || msg->step < min_step) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "图像步长不足: encoding=%s width=%u step=%u required=%zu",
                    encoding.c_str(), msg->width, msg->step, min_step);
                return;
            }
            cv::Mat raw(static_cast<int>(msg->height), static_cast<int>(msg->width),
                        type, const_cast<unsigned char *>(msg->data.data()), msg->step);
            cv::Mat image;
            if (encoding == sensor_msgs::image_encodings::RGB8) {
                cv::cvtColor(raw, image, cv::COLOR_RGB2BGR);
            } else if (encoding == sensor_msgs::image_encodings::RGBA8) {
                cv::cvtColor(raw, image, cv::COLOR_RGBA2BGR);
            } else if (encoding == sensor_msgs::image_encodings::BGRA8) {
                cv::cvtColor(raw, image, cv::COLOR_BGRA2BGR);
            } else if (encoding == sensor_msgs::image_encodings::MONO16 || encoding == "16UC1") {
                raw.convertTo(image, CV_8UC1, 1.0 / 256.0);
                cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
            } else if (encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
                double min_value = 0.0, max_value = 0.0;
                cv::minMaxLoc(raw, &min_value, &max_value);
                if (!std::isfinite(min_value) || !std::isfinite(max_value) || max_value <= min_value) {
                    image = cv::Mat(raw.rows, raw.cols, CV_8UC1, cv::Scalar(0));
                } else {
                    raw.convertTo(image, CV_8UC1, 255.0 / (max_value - min_value),
                                  -min_value * 255.0 / (max_value - min_value));
                }
                cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
            } else if (type == CV_8UC1) {
                cv::cvtColor(raw, image, cv::COLOR_GRAY2BGR);
            } else {
                image = raw.clone();
            }
            if (image.empty()) return;
            {
                std::lock_guard<std::mutex> lock(image_mutex_);
                latest_image_ = image.clone();
                latest_image_time_ = std::chrono::steady_clock::now();
            }
            if (!image_ready_logged_) {
                image_ready_logged_ = true;
                RCLCPP_INFO(
                    get_logger(), "已收到有效相机图像: %ux%u, encoding=%s",
                    msg->width, msg->height, encoding.c_str());
            }
        } catch (const cv::Exception & ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "OpenCV 图像处理失败: %s", ex.what());
        } catch (const std::exception & ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "图像处理失败: %s", ex.what());
        }
    }

    void tick() {
        if (stopped_ || state_ == State::Stopped || waypoints_.empty()) return;
        if (state_ == State::WaitingNav) {
            if (!nav_client_->action_server_is_ready()) return;
            if (!tf_buffer_.canTransform("map", "base_link", tf2::TimePointZero)) {
                RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "等待定位完成，暂不发送巡检目标: map -> base_link 变换不可用");
                return;
            }
            sendCurrentGoal();
            return;
        }
        if (state_ == State::Navigating &&
            std::chrono::steady_clock::now() - nav_started_ > std::chrono::duration<double>(goal_timeout_sec_)) {
            const auto old_handle = goal_handle_;
            ++goal_token_;
            goal_handle_.reset();
            if (old_handle) nav_client_->async_cancel_goal(old_handle);
            double distance = std::numeric_limits<double>::infinity();
            if (isNearCurrentWaypoint(distance)) {
                RCLCPP_WARN(
                    get_logger(), "巡检点 [%s] 导航超时，但已距目标 %.2f 米，按到达处理",
                    waypoints_[index_].name.c_str(), distance);
                handleArrival();
            } else {
                RCLCPP_ERROR(get_logger(), "巡检点 [%s] 导航超时，跳过", waypoints_[index_].name.c_str());
                finishWaypoint();
            }
            return;
        }
        if (state_ == State::WaitingImage) {
            bool have_image = false;
            {
                std::lock_guard<std::mutex> lock(image_mutex_);
                have_image = !latest_image_.empty() && latest_image_time_ >= image_started_;
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
            !speech_request_sent_ && speech_client_->service_is_ready()) {
            sendSpeechRequest();
        }
        if (state_ == State::WaitingSpeech &&
            std::chrono::steady_clock::now() - speech_started_ > std::chrono::duration<double>(speech_timeout_sec_)) {
            RCLCPP_WARN(get_logger(), "语音服务超时，继续巡检");
            finishWaypoint();
        }
    }
    /**
     * @brief 发送巡检点导航目标
    */
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
                double distance = std::numeric_limits<double>::infinity();
                if (isNearCurrentWaypoint(distance)) {
                    RCLCPP_WARN(
                        get_logger(), "巡检点 [%s] 导航未成功结束，但已距目标 %.2f 米，按到达处理",
                        waypoints_[index_].name.c_str(), distance);
                    handleArrival();
                } else {
                    RCLCPP_ERROR(
                        get_logger(), "巡检点 [%s] 导航失败且未到达，跳过",
                        waypoints_[index_].name.c_str());
                    finishWaypoint();
                }
            }
        };
        nav_client_->async_send_goal(goal, options);
    }

    bool isNearCurrentWaypoint(double & distance) {
        try {
            const auto transform = tf_buffer_.lookupTransform(
                "map", "base_link", tf2::TimePointZero);
            const auto & point = waypoints_[index_];
            const double dx = transform.transform.translation.x - point.x;
            const double dy = transform.transform.translation.y - point.y;
            distance = std::hypot(dx, dy);
            return std::isfinite(distance) && distance <= arrival_tolerance_;
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(get_logger(), "检查巡检点距离失败: %s", ex.what());
            return false;
        }
    }
    /**
     * @brief 到达巡检点
    */
    void handleArrival() {
        state_ = State::WaitingImage;
        image_started_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(get_logger(), "已到达巡检点 [%s]，等待保存新图像", waypoints_[index_].name.c_str());
    }
    /**
     * @brief 语音播报
    */
    void beginSpeech() {
        const auto & point = waypoints_[index_];
        if (point.text.empty()) {
            finishWaypoint();
            return;
        }
        state_ = State::WaitingSpeech;
        speech_started_ = std::chrono::steady_clock::now();
        speech_request_sent_ = false;
        if (speech_client_->service_is_ready()) sendSpeechRequest();
        else RCLCPP_WARN(get_logger(), "语音服务尚未就绪，将等待 %.1f 秒", speech_timeout_sec_);
    }

    void sendSpeechRequest() {
        if (speech_request_sent_ || state_ != State::WaitingSpeech) return;
        auto request = std::make_shared<SpeechText::Request>();
        request->text = waypoints_[index_].text;
        const auto token = goal_token_;
        speech_request_sent_ = true;
        RCLCPP_INFO(get_logger(), "播报巡检点 [%s]: %s", waypoints_[index_].name.c_str(), request->text.c_str());
        speech_client_->async_send_request(request, [this, token](
        std::shared_future<SpeechText::Response::SharedPtr> future) {
            if (token != goal_token_ || state_ != State::WaitingSpeech) return;
            try {
                const auto response = future.get();
                if (!response->success) RCLCPP_WARN(get_logger(), "语音播报失败: %s", response->message.c_str());
                else RCLCPP_INFO(get_logger(), "语音播报完成");
            } catch (const std::exception & ex) {
                RCLCPP_WARN(get_logger(), "语音服务响应异常: %s", ex.what());
            }
            finishWaypoint();
        });
    }
    /**
     * @brief 保存图像
    */
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
        const auto now = std::chrono::system_clock::now();
        const auto time = std::chrono::system_clock::to_time_t(now);
        const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        std::tm tm{};
        localtime_r(&time, &tm);
        std::ostringstream filename;
        filename << image_save_dir_ << "/" << name << "_"
                 << std::put_time(&tm, "%Y%m%d_%H%M%S") << "_"
                 << std::setfill('0') << std::setw(3) << milliseconds.count() << ".jpg";
        try {
            if (!cv::imwrite(filename.str(), image)) {
                RCLCPP_ERROR(get_logger(), "保存图像失败: %s", filename.str().c_str());
                return false;
            }
        } catch (const cv::Exception & ex) {
            RCLCPP_ERROR(get_logger(), "OpenCV 保存图像失败: %s", ex.what());
            return false;
        }
        RCLCPP_INFO(get_logger(), "已保存巡检图像: %s", filename.str().c_str());
        return true;
    }
    /**
     * 完成巡检
    */
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
    double goal_timeout_sec_{180.0}, image_wait_timeout_sec_{2.0}, speech_timeout_sec_{60.0};
    double arrival_tolerance_{0.35};
    size_t index_{0};
    State state_{State::WaitingNav};
    uint64_t goal_token_{0};
    bool speech_request_sent_{false};
    bool image_ready_logged_{false};
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
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PatrolController>());
    rclcpp::shutdown();
    return 0;
}
