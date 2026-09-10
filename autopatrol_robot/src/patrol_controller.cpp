/**
 * @file patrol_controller.cpp
 * @brief 自动巡检任务的导航、拍照与语音编排节点。
 *
 * 单个巡检点依次经历：等待导航、执行导航、等待新图像、语音播报。
 * Nav2 负责运动，相机节点提供图像，TTS 服务负责合成并播放语音。
 *
 * 关键设计约束：
 * - 使用真实单调时钟判断超时，避免仿真时间暂停影响看门狗；
 * - 使用 ROS 时钟生成导航时间戳，保证仿真 TF 能正确匹配；
 * - 使用 goal_token_ 丢弃取消或超时后到达的旧异步回调；
 * - 使用互斥锁保护相机回调与状态机共享的最新图像；
 * - 任一巡检点失败只影响当前点，不永久阻塞后续任务。
 *
 * 主要参数包括巡检点文件、图像话题、保存目录、语音服务、
 * 是否循环，以及导航、图像、语音三个阶段的超时值。
 * arrival_tolerance 用于导航异常结束后的实际距离补偿判断，
 * 它不替代 Nav2 自身的目标容差配置。
 *
 * 图像处理先校验尺寸、步长和数据长度，再构造 cv::Mat。
 * 支持常见彩色、灰度和浮点深度编码，最终转换为可保存格式。
 * 只保存机器人到达巡检点之后接收到的新帧，避免误用途中缓存。
 *
 * 导航失败或超时时，若 TF 距离已经小于容差则仍按到达处理。
 * 图像、保存或语音失败会记录日志并继续执行下一个巡检点。
 */
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

/**
 * @brief 一个有序巡检目标。
 *
 * 坐标位于 map 坐标系。
 * yaw 是绕 Z 轴旋转的弧度值。
 * text 是到达后交给语音服务的完整文本。
 */
struct Waypoint {
    std::string name;
    double x{};
    double y{};
    double yaw{};
    std::string text;
};

/**
 * @brief 以非阻塞状态机编排整轮巡检。
 *
 * 200 ms 定时器驱动状态迁移。
 * action 和 service 使用异步回调，因此 executor 不会因导航或播报而阻塞。
 * 语音服务本身在响应前完成音频播放，所以响应成功即表示播报结束。
 *
 */
class PatrolController final : public rclcpp::Node {
public:
    /// Nav2 的单目标导航
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    /// 巡检点导航目标
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    /// 语音服务数据结构
    using SpeechText = autopatrol_robot::srv::SpeechText;

    /**
     * @brief 创建巡检控制器并准备所有 ROS 通信实体。
     *
     * 参数在创建订阅者和客户端之前读取，确保自定义名称从第一次发现开始生效。
     * 配置加载失败时保留节点以输出诊断信息，但 stopped_ 会阻止任务执行。
     */
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
    /**
     * @brief 单个巡检点的任务阶段。
     *
     * WaitingNav 表示尚未发送目标，可等待 Nav2 或定位就绪。
     * Navigating 表示 action 已发送，正在等待接受或最终结果。
     * WaitingImage 表示已到达，正在等待一帧新的相机图像。
     * WaitingSpeech 表示准备调用或正在等待 SpeechText 服务。
     * Stopped 是非循环任务完成后的终态。
     *
     * 正常迁移为：
     * WaitingNav -> Navigating -> WaitingImage -> WaitingSpeech -> WaitingNav。
     * 错误分支可从 Navigating、WaitingImage 或 WaitingSpeech 直接进入下一点。
     */
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
     * @note 只有全部巡检点校验通过，任务才允许启动。
     *
     * 校验在启动时一次完成，避免导航过程中才发现缺少字段。
     * names 集合同时完成非空名称的唯一性检查。
     * YAML 异常统一转为 ROS 错误日志，不让构造函数异常退出。
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
     * @param msg 相机发布的原始 sensor_msgs/Image。
     *
     * 回调完成三层防御：消息边界校验、encoding 到 cv::Mat 类型映射、
     * 统一颜色/位深转换。只有成功转换的图像才替换 latest_image_。
     *
     * 100 MiB 上限防止损坏的 width、height 或 step 导致过量分配。
     * 对乘法先做除法上限检查，避免 size_t 乘法溢出。
     *
     * RGB/RGBA 会转换为 OpenCV 默认的 BGR。
     * 16 位灰度按完整 16 位范围线性缩放到 8 位。
     * 32 位浮点深度按当前帧最小值和最大值归一化用于留档。
     * 深度帧没有有限动态范围时生成全黑图像，而不是抛出异常。
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
            // 不强制通过 cv_bridge 转换所有消息。Gazebo 可能发布彩色、
            // 灰度或浮点深度图；损坏的编码或步长可能使 cv_bridge 在
            // 输出有效错误前就构造出非法的 cv::Mat。
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

    /**
     * @brief 推进巡检状态机并执行各阶段的超时策略。
     *
     * 本函数由墙上时钟定时器每 200 ms 调用。
     * 它只发起异步操作，不等待 action 或 service 完成。
     * 每个分支处理一个状态后立即返回，保证单次 tick 工作量有界。
     *
     * WaitingNav 同时要求 action server 和定位 TF 可用。
     * Navigating 使用 steady_clock 检查目标超时。
     * WaitingImage 要求帧接收时间不早于到达时间。
     * WaitingSpeech 会在服务稍晚上线时补发请求，并监控总超时。
     */
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
     *
     * 将二维 yaw 转换为绕 Z 轴旋转的单位四元数。
     * 目标使用 map 坐标系和 ROS clock 时间戳。
     *
     * goal_token_ 标识本次异步操作。
     * 当任务因超时进入下一阶段后，旧 action 回调仍可能到达；
     * token 不匹配时必须忽略，否则旧结果会污染新巡检点状态。
     *
     * Nav2 返回非 SUCCEEDED 不一定意味着机器人离目标很远。
     * 局部规划器可能在容差或进度检查边界处失败，
     * 因此结果失败时再用 TF 计算实际平面距离。
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

    /**
     * @brief 判断机器人是否已进入当前巡检点的补偿容差。
     * @param[out] distance map 平面内机器人到目标的欧氏距离。
     * @return 距离有限且不大于 arrival_tolerance_ 时返回 true。
     *
     * 查询最新 TF，而不是 action feedback 中的估计距离，
     * 保证导航超时和最终结果回调可以共用同一判断依据。
     * TF 不可用时保守返回 false，避免把未知位置误判为到达。
     */
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
     *
     * 记录进入 WaitingImage 的 steady_clock 时间。
     * 图像回调记录的接收时间必须晚于该时间，
     * 从而确保保存的是到达现场后的新帧而非导航途中的缓存帧。
     */
    void handleArrival() {
        state_ = State::WaitingImage;
        image_started_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(get_logger(), "已到达巡检点 [%s]，等待保存新图像", waypoints_[index_].name.c_str());
    }
    /**
     * @brief 语音播报
     *
     * 空文本不调用服务，直接完成当前点。
     * 服务已经就绪时立即发送；否则保留 WaitingSpeech，
     * 后续 tick 会在服务被发现后发送同一个请求。
     * speech_started_ 覆盖等待服务和等待响应的总耗时。
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

    /**
     * @brief 异步提交当前巡检点的语音文本。
     *
     * speech_request_sent_ 防止 200 ms tick 重复提交。
     * 回调再次核对 token 和状态，避免超时后的迟到响应重复推进 index_。
     * 无论服务返回失败还是抛出异常，当前点最终都会结束。
     */
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
     * @param name 当前巡检点名称，用作文件名前缀。
     * @return 目录创建且 JPEG 写入成功时返回 true。
     *
     * 加锁区域只复制 cv::Mat，磁盘 I/O 在锁外完成，
     * 因此相机回调不会被慢磁盘长时间阻塞。
     * 文件名包含毫秒，降低快速重复巡检时覆盖旧文件的概率。
     * create_directories 使用 error_code，避免文件系统异常穿过 ROS 回调边界。
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
     * @brief 完成当前巡检点并选择下一个任务。
     *
     * 非循环模式在最后一点后进入 Stopped。
     * 循环模式把索引归零并重新进入 WaitingNav。
     * 函数可由多个失败路径调用，因此首先保护 Stopped 终态。
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

    /// 启动时从 YAML 加载且运行期间只读的有序巡检点。
    std::vector<Waypoint> waypoints_;
    /// 外部资源路径和 ROS 接口名称。
    std::string waypoints_file_, image_topic_, image_save_dir_, speech_service_, navigate_action_;
    /// 循环策略以及配置失败后的总停止标记。
    bool loop_enabled_{true}, stopped_{false};
    /// 三个阶段的墙上时钟超时，单位均为秒。
    double goal_timeout_sec_{180.0}, image_wait_timeout_sec_{2.0}, speech_timeout_sec_{60.0};
    /// action 异常结束时接受目标到机器人的最大平面距离。
    double arrival_tolerance_{0.35};
    /// 当前巡检点在 waypoints_ 中的下标。
    size_t index_{0};
    /// 当前任务阶段，仅由状态机和异步结果回调修改。
    State state_{State::WaitingNav};
    /// 单调递增的 action 世代号，用于拒绝迟到回调。
    uint64_t goal_token_{0};
    /// 当前 WaitingSpeech 阶段是否已经发出服务请求。
    bool speech_request_sent_{false};
    /// 限制“相机已就绪”日志只输出一次。
    bool image_ready_logged_{false};
    /// 当前正在执行的 Nav2 目标句柄；超时取消后清空。
    GoalHandle::SharedPtr goal_handle_;
    /// 导航和语音阶段的 steady_clock 起点。
    std::chrono::steady_clock::time_point nav_started_, speech_started_;
    /// 进入 WaitingImage 的时刻，用于排除旧图像。
    std::chrono::steady_clock::time_point image_started_;
    /// 最近一帧已转换为可保存格式的图像。
    cv::Mat latest_image_;
    /// latest_image_ 对应的本地接收时间。
    std::chrono::steady_clock::time_point latest_image_time_;
    /// 保护 latest_image_ 和 latest_image_time_ 的互斥锁。
    std::mutex image_mutex_;
    /// 相机图像订阅者。
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    /// Nav2 NavigateToPose action 客户端。
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    /// TTS 同步播放服务客户端。
    rclcpp::Client<SpeechText>::SharedPtr speech_client_;
    /// 驱动状态机的 200 ms 墙上时钟定时器。
    rclcpp::TimerBase::SharedPtr timer_;
    /// 查询机器人实际 map 坐标的 TF 缓存。
    tf2_ros::Buffer tf_buffer_;
    /// 自动把 /tf 和 /tf_static 填充到 tf_buffer_。
    tf2_ros::TransformListener tf_listener_;
};

/**
 * @brief ROS 2 进程入口。
 *
 * 初始化 rclcpp 后使用默认单线程 executor 运行控制器，
 * shutdown 会在外部终止信号或 ROS context 结束后执行。
 */
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PatrolController>());
    rclcpp::shutdown();
    return 0;
}
