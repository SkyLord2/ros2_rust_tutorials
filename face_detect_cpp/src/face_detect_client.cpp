#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <learning_interface/srv/face_detector.hpp>

using namespace std::chrono_literals;

class FaceDetectClient : public rclcpp::Node
{
private:
    rclcpp::Client<learning_interface::srv::FaceDetector>::SharedPtr client_;
public:
    FaceDetectClient(std::string name) : Node(name) {
        client_ = this->create_client<learning_interface::srv::FaceDetector>("face_detect");
    }
    void send_request_async() {
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "等待服务过程中被中断，退出。");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "正在等待人脸检测服务端上线...");
        }

        cv::Mat local_image = cv::imread("/home/cds/Code/ros2/dev_ws/src/ros2_rust_tutorials/face_detect_cpp/face.jpg");

        auto image = cv_bridge::CvImage(
            std_msgs::msg::Header(),
            sensor_msgs::image_encodings::BGR8,
            local_image
        );

        auto request = std::make_shared<learning_interface::srv::FaceDetector::Request>();
        request->image = *image.toImageMsg();
        
        client_->async_send_request(
            request, 
            [this, local_image](rclcpp::Client<learning_interface::srv::FaceDetector>::SharedFuture future) {
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "异步收到响应：耗时: %.2f ms，检测到 %d 张人脸", response->cost_time, response->number);

                cv::Mat result_image = local_image.clone();

                for (int i = 0; i < response->number; i++)
                {
                    cv::Point top_left(response->left[i], response->top[i]);
                    cv::Point bottom_right(response->right[i], response->bottom[i]);

                    cv::rectangle(result_image, top_left, bottom_right, cv::Scalar(0, 255, 0), 2);

                    std::string label = "Face " + std::to_string(i + 1);
                    cv::putText(result_image, label, 
                                cv::Point(response->left[i], response->top[i] - 6),
                                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                }

                std::string summary = "Faces: " + std::to_string(response->number) + 
                                     " | Time: " + std::to_string(response->cost_time) + " ms";
                cv::putText(result_image, summary, cv::Point(15, 30),
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);

                cv::imshow("Face Detection Result", result_image);
                cv::waitKey(0);
            }
        );
    }
    ~FaceDetectClient() {

    }
};


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FaceDetectClient>("face_detect_client");

    node->send_request_async();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
