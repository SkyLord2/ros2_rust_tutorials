/**
 * @作者: 古月居(www.guyuehome.com)
 * @说明: ROS2 TF示例-通过坐标变化实现海龟跟随功能
 */

#include <cmath>
#include <chrono>
#include <memory>
#include <string>

// ROS2 C++接口库 (对应 rclpy)
#include "rclcpp/rclcpp.hpp"
// TF坐标变换的异常类
#include "tf2/exceptions.h"
// 存储坐标变换信息的缓冲类
#include "tf2_ros/buffer.h"
// 监听坐标变换的监听器类
#include "tf2_ros/transform_listener.h"
// ROS2 速度控制消息
#include "geometry_msgs/msg/twist.hpp"
// 海龟生成的服务接口
#include "turtlesim/srv/spawn.hpp"

using namespace std::chrono_literals;

// ROS2 节点类
class TurtleFollowing : public rclcpp::Node
{
public:
    // ROS2节点父类初始化
    explicit TurtleFollowing(const std::string & name) : Node(name)
    {
        // 创建一个源坐标系名的参数
        this->declare_parameter<std::string>("source_frame", "turtle1");
        
        // 优先使用外部设置的参数值，否则用默认值
        this->get_parameter("source_frame", source_frame_);

        // 创建保存坐标变换信息的缓冲区
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        
        // 创建坐标变换的监听器
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 创建一个请求产生海龟的客户端
        spawner_ = this->create_client<turtlesim::srv::Spawn>("spawn");
        
        // 标志位初始化
        turtle_spawning_service_ready_ = false; // 是否已经请求海龟生成服务的标志位
        turtle_spawned_ = false;                // 海龟是否产生成功的标志位

        // 创建跟随运动海龟的速度话题
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 1);

        // 创建一个固定周期的定时器，控制跟随海龟的运动
        // 原 Python 代码中使用了 1.0 秒周期
        timer_ = this->create_wall_timer(
            1.0s, std::bind(&TurtleFollowing::on_timer, this));
    }

private:
    void on_timer()
    {
        std::string from_frame_rel = source_frame_; // 源坐标系
        std::string to_frame_rel = "turtle2";       // 目标坐标系

        if (turtle_spawning_service_ready_) { // 如果已经请求海龟生成服务
            if (turtle_spawned_) {            // 如果跟随海龟已经生成
                geometry_msgs::msg::TransformStamped trans;
                try {
                    // 获取ROS系统的当前时间 / 监听当前时刻源坐标系到目标坐标系的坐标变换
                    // C++ 特有：使用 tf2::TimePointZero 获取最新有效变换
                    trans = tf_buffer_->lookupTransform(
                        to_frame_rel,
                        from_frame_rel,
                        tf2::TimePointZero);
                } catch (const tf2::TransformException & ex) { // 如果坐标变换获取失败，进入异常报告
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Could not transform %s to %s: %s",
                        to_frame_rel.c_str(), from_frame_rel.c_str(), ex.what());
                    return;
                }

                geometry_msgs::msg::Twist msg; // 创建速度控制消息

                double scale_rotation_rate = 1.0; // 根据海龟角度，计算角速度
                // C++ 中使用 std::atan2 计算反正切
                msg.angular.z = scale_rotation_rate * std::atan2(
                    trans.transform.translation.y,
                    trans.transform.translation.x);

                double scale_forward_speed = 0.5; // 根据海龟距离，计算线速度
                // C++ 中使用 std::sqrt 计算平方根，也可配合 std::pow 
                msg.linear.x = scale_forward_speed * std::sqrt(
                    std::pow(trans.transform.translation.x, 2) +
                    std::pow(trans.transform.translation.y, 2));

                publisher_->publish(msg); // 发布速度指令，海龟跟随运动
            } else { // 如果跟随海龟没有生成
                // 查看海龟是否生成：C++ 中通过判断 shared_future 的状态（非阻塞查看）
                if (result_future_.valid() && 
                    result_future_.wait_for(0s) == std::future_status::ready) {
                    
                    auto response = result_future_.get();
                    RCLCPP_INFO(this->get_logger(), "Successfully spawned %s", response->name.c_str());
                    turtle_spawned_ = true;
                } else { // 依然没有生成跟随海龟
                    RCLCPP_INFO(this->get_logger(), "Spawn is not finished");
                }
            }
        } else { // 如果没有请求海龟生成服务
            if (spawner_->service_is_ready()) { // 如果海龟生成服务器已经准备就绪
                auto request = std::make_shared<turtlesim::srv::Spawn::Request>(); // 创建一个请求的数据
                request->name = "turtle2"; // 设置请求数据的内容，包括海龟名、xy位置、姿态
                request->x = 4.0;          
                request->y = 2.0;         
                request->theta = 0.0;     

                // 发送服务请求，并获取一个 shared_future 以供后续检查
                result_future_ = spawner_->async_send_request(request).future.share();
                turtle_spawning_service_ready_ = true; // 设置标志位，表示已经发送请求
            } else {
                RCLCPP_INFO(this->get_logger(), "Service is not ready"); // 海龟生成服务器还没准备就绪的提示
            }
        }
    }

    // 类成员变量声明
    std::string source_frame_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawner_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    bool turtle_spawning_service_ready_;
    bool turtle_spawned_;
    // 用于保存异步服务调用的返回结果 (对应 Python 中的 self.result)
    std::shared_future<turtlesim::srv::Spawn_Response::SharedPtr> result_future_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);                                       // ROS2 Python接口初始化 (映射至C++)
    auto node = std::make_shared<TurtleFollowing>("turtle_following"); // 创建ROS2节点对象并进行初始化
    rclcpp::spin(node);                                             // 循环等待ROS2退出
    // 销毁节点对象：C++的智能指针在 main 退出时自动销毁
    rclcpp::shutdown();                                             // 关闭ROS2 Python接口 (映射至C++)
    return 0;
}