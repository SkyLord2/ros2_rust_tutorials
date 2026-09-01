#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <angles/angles.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

class DynamicTFBroadcaster : public rclcpp::Node
{
private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
private:
    void publish_tf() {
        RCLCPP_INFO(this->get_logger(), "Publishing transform");
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->get_clock()->now();
        transformStamped.header.frame_id = "camera_link";
        transformStamped.child_frame_id = "bottle_link";

        transformStamped.transform.translation.x = 0.2;
        transformStamped.transform.translation.y = 0.3;
        transformStamped.transform.translation.z = 0.5;

        tf2::Quaternion quat;
        double roll = angles::from_degrees(0.0);
        double pitch = angles::from_degrees(0.0);
        double yaw = angles::from_degrees(0.0);
        quat.setRPY(roll, pitch, yaw);
        transformStamped.transform.rotation.x = quat.x();
        transformStamped.transform.rotation.y = quat.y();
        transformStamped.transform.rotation.z = quat.z();
        transformStamped.transform.rotation.w = quat.w();

        tf_broadcaster_->sendTransform(transformStamped);
    }
    void publish_tf_recycle() {
        timer_ = this->create_wall_timer(
            100ms,
            [this]() {
                publish_tf();
            }
        );
    }
public:
    DynamicTFBroadcaster(std::string name) : Node(name)
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        publish_tf_recycle();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicTFBroadcaster>("dynamic_broadcaster");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}