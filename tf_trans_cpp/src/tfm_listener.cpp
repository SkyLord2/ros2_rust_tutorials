#include <memory>
#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/exceptions.h>

using namespace std::chrono_literals;

class TFListener : public rclcpp::Node 
{
private:
    std::string source_frame_;
    std::string target_frame_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;

private:
    void on_timer()
    {
        geometry_msgs::msg::TransformStamped trans;
        try {
            // 获取ROS系统的当前时间 / 监听当前时刻源坐标系到目标坐标系的坐标变换
            // C++ 特有技巧：使用 tf2::TimePointZero 代表时间 "0"，等同于获取最新时刻的坐标变换
            trans = tf_buffer_->lookupTransform(
                target_frame_,
                source_frame_,
                tf2::TimePointZero);
        }
        // 如果坐标变换获取失败，进入异常报告
        catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(
                this->get_logger(),
                "Could not transform %s to %s: %s",
                target_frame_.c_str(), source_frame_.c_str(), ex.what()); //
            return;
        }

        // 获取位置信息
        auto pos = trans.transform.translation;
        
        // 获取姿态信息（四元数）
        auto quat = trans.transform.rotation;

        // C++特有转换流程：
        // 1. 将收到的消息提取到 tf2 库原生的四元数容器中
        tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
        // 2. 将四元数转化为 3x3 旋转矩阵
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        // 3. 从旋转矩阵中提取欧拉角 (对应 Python 的 euler_from_quaternion)
        m.getRPY(roll, pitch, yaw);

        // 打印信息：输出原点与目标的位移和姿态
        RCLCPP_INFO(
            this->get_logger(),
            "Get %s --> %s transform: [%.9f, %.9f, %.9f] [%.9f, %.9f, %.9f]",
            source_frame_.c_str(), target_frame_.c_str(),
            pos.x, pos.y, pos.z,
            roll, pitch, yaw); //
    }
public:
    explicit TFListener(std::string name) : Node(name)
    {
        this->declare_parameter<std::string>("source_frame", "bottle_link");
        this->get_parameter("source_frame", source_frame_);

        this->declare_parameter<std::string>("target_frame", "base_link");
        this->get_parameter("target_frame", target_frame_);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        timer_ = this->create_wall_timer(1s, std::bind(&TFListener::on_timer, this));
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFListener>("tfm_listener");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}