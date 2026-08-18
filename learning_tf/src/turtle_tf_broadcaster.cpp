#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <turtlesim/msg/pose.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class TurtlrTFBroadcaster : public rclcpp::Node
{
private:
    std::string turtle_name_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
private:
    void turtle_pose_calback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped transform;
        // 设置坐标变换消息的时间戳
        transform.header.stamp = this->get_clock()->now();
        // 设置一个坐标变换的源坐标系
        transform.header.frame_id = "world";
        // 设置一个坐标变换的目标坐标系
        transform.child_frame_id = turtle_name_;
        // 设置坐标变换中的X、Y、Z向的平移
        // 从 msg 智能指针中提取 x 和 y
        transform.transform.translation.x = msg->x;
        transform.transform.translation.y = msg->y;
        transform.transform.translation.z = 0.0;

        // 将欧拉角转换为四元数（roll, pitch, yaw）
        tf2::Quaternion q;
        // C++ 特有：直接调用 tf2::Quaternion 的 setRPY 方法
        q.setRPY(0.0, 0.0, msg->theta);
        // 设置坐标变换中的X、Y、Z向的旋转（四元数）
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        // 广播坐标变换，海龟位置变化后，将及时更新坐标变换信息
        transform_broadcaster_->sendTransform(transform);
    }
public:
    TurtlrTFBroadcaster(std::string name) : Node(name)
    {
        this->declare_parameter<std::string>("turtlename", "turtle");
        this->get_parameter("turtlename", turtle_name_);

        transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        std::string topic_name = "/" + turtle_name_ + "/pose";

        subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            topic_name, 
            1,
            std::bind(&TurtlrTFBroadcaster::turtle_pose_calback, this, std::placeholders::_1)
        );
    }
    ~TurtlrTFBroadcaster()
    {

    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TurtlrTFBroadcaster>("turtle_tf_broadcaster");
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}