#include <chrono>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using NavigationAction = nav2_msgs::action::FollowWaypoints;  // 定义导航动作类型为FollowWaypoints

class WaypointFollower : public rclcpp::Node {
public:
    using NavigationActionClient = rclcpp_action::Client<NavigationAction>;  // 定义导航动作客户端类型
    using NavigationActionGoalHandle =
        rclcpp_action::ClientGoalHandle<NavigationAction>;  // 定义导航动作目标句柄类型

    WaypointFollower() : Node("waypoint_follow_client", rclcpp::NodeOptions().append_parameter_override("use_sim_time", true)) {
        // 创建导航动作客户端
        action_client_ = rclcpp_action::create_client<NavigationAction>(
            this, "follow_waypoints");
    }

    geometry_msgs::msg::PoseStamped createWaypoint(double x, double y, double w = 1.0) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->get_clock()->now();
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = w;
        return pose;
    }

    void sendGoal() {
        // 等待导航动作服务器上线，等待时间为5秒
        while (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(get_logger(), "等待服务上线时被系统中断。");
                return;
            }
            RCLCPP_INFO(get_logger(), "等待Action服务上线。");
        }
        // 设置导航目标点
        auto goal_msg = NavigationAction::Goal();
        goal_msg.poses.push_back(createWaypoint(2.0, 1.0, 1.0));
        goal_msg.poses.push_back(createWaypoint(0.0, 1.0, 1.0));
        goal_msg.poses.push_back(createWaypoint(0.0, 0.0, 1.0));

        auto send_goal_options =
            rclcpp_action::Client<NavigationAction>::SendGoalOptions();
        // 设置请求目标结果回调函数
        send_goal_options.goal_response_callback =
            [this](NavigationActionGoalHandle::SharedPtr goal_handle) {
                if (goal_handle) {
                    RCLCPP_INFO(get_logger(), "目标点已被服务器接收");
                } else {
                    RCLCPP_ERROR(get_logger(), "目标点未被服务器接收");
                }
            };
        // 设置移动过程反馈回调函数
        send_goal_options.feedback_callback =
            [this](
                NavigationActionGoalHandle::SharedPtr goal_handle,
                const std::shared_ptr<const NavigationAction::Feedback> feedback) {
                (void)goal_handle;  // 假装调用，避免 warning: unused
                RCLCPP_INFO(this->get_logger(), "【反馈】当前正在前往第 %d 个目标点 (索引从 0 开始)", 
                            feedback->current_waypoint);
            };
        // 设置执行结果回调函数
        send_goal_options.result_callback =
            [this](const NavigationActionGoalHandle::WrappedResult& result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(), "成功到达所有目标点！");
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(this->get_logger(), "任务中止！错过的点数量: %zu", 
                                     result.result->missed_waypoints.size());
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_WARN(this->get_logger(), "任务被取消");
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "未知结果代码");
                        break;
                }
            };
        // 发送导航目标点
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    NavigationActionClient::SharedPtr action_client_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointFollower>();
  node->sendGoal();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
