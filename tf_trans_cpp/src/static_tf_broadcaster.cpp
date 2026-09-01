#include <rclcpp/rclcpp.hpp>
#include <angles/angles.h>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class StaticTFBroadcaster : public rclcpp::Node
{
private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

private:
    void publish_tf()
    {
        geometry_msgs::msg::TransformStamped static_transformStamped;
        static_transformStamped.header.stamp = this->get_clock()->now();
        static_transformStamped.header.frame_id = "base_link";
        static_transformStamped.child_frame_id = "camera_link";

        static_transformStamped.transform.translation.x = 0.5;
        static_transformStamped.transform.translation.y = 0.3;
        static_transformStamped.transform.translation.z = 0.6;

        tf2::Quaternion quat;
        double roll = angles::from_degrees(180.0);
        double pitch = angles::from_degrees(0.0);
        double yaw = angles::from_degrees(0.0);
        quat.setRPY(roll, pitch, yaw);
        static_transformStamped.transform.rotation.x = quat.x();
        static_transformStamped.transform.rotation.y = quat.y();
        static_transformStamped.transform.rotation.z = quat.z();
        static_transformStamped.transform.rotation.w = quat.w();

        static_broadcaster_->sendTransform(static_transformStamped);
    }
public:
    StaticTFBroadcaster(std::string name) : Node(name)
    {
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        publish_tf();
        RCLCPP_INFO(this->get_logger(), "Static TF Broadcaster started.");
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StaticTFBroadcaster>("static_broadcaster");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}