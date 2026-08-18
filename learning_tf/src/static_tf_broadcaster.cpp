#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class StaticTFBroadcaster : public rclcpp::Node
{
private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
public:
    explicit StaticTFBroadcaster(const std::string name) : Node(name) 
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        // 创建一个坐标变换的消息对象
        geometry_msgs::msg::TransformStamped static_transformStamped;
        // 设置坐标变换消息的时间戳
        static_transformStamped.header.stamp = this->get_clock()->now();
        // 设置一个坐标变换的源坐标系
        static_transformStamped.header.frame_id = "world";
        // 设置一个坐标变换的目标坐标系
        static_transformStamped.child_frame_id = "house";

        // 设置坐标变换中的X、Y、Z向的平移
        static_transformStamped.transform.translation.x = 10.0;
        static_transformStamped.transform.translation.y = 5.0;
        static_transformStamped.transform.translation.z = 0.0;

        // 将欧拉角转换为四元数（roll, pitch, yaw）
        tf2::Quaternion quat;
        // C++ 接口直接调用 setRPY 即可完成欧拉角到四元数的计算
        quat.setRPY(0.0, 0.0, 0.0);
        // 设置坐标变换中的X、Y、Z向的旋转（四元数）
        static_transformStamped.transform.rotation.x = quat.x();
        static_transformStamped.transform.rotation.y = quat.y();
        static_transformStamped.transform.rotation.z = quat.z();
        static_transformStamped.transform.rotation.w = quat.w();

        // 广播静态坐标变换，广播后两个坐标系的位置关系保持不变
        tf_broadcaster_->sendTransform(static_transformStamped);
    }   
};

// ROS2节点主入口main函数
int main(int argc, char * argv[])
{
    // ROS2 C++接口初始化（等同于 ROS2 Python接口初始化）[cite: 17]
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StaticTFBroadcaster>("static_tf_broadcaster");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}