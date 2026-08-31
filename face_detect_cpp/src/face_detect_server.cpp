#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <learning_interface/srv/face_detector.hpp>
#include <algorithm>

class FaceDetectServer : public rclcpp::Node
{
private:
    rclcpp::Service<learning_interface::srv::FaceDetector>::SharedPtr service_;
    cv::Ptr<cv::FaceDetectorYN> detector_;
    std::string model_path_;

    // 推理基准长边尺寸（如 640 或 800）
    const int TARGET_MAX_DIM = 640;

private:
    void handle_face_detect(
        const std::shared_ptr<learning_interface::srv::FaceDetector::Request> request,
        std::shared_ptr<learning_interface::srv::FaceDetector::Response> response
    ) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(request->image, sensor_msgs::image_encodings::BGR8);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 转换图像失败: %s", e.what());
            response->number = 0;
            response->cost_time = 0.0f;
            return;
        }

        cv::Mat frame = cv_ptr->image;
        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "接收到的图像为空！");
            response->number = 0;
            response->cost_time = 0.0f;
            return;
        }

        // 1. 等比例缩放（保持图像长宽比不变，防止脸部被拉伸变形）
        float scale = static_cast<float>(TARGET_MAX_DIM) / std::max(frame.cols, frame.rows);
        int target_w = (static_cast<int>(frame.cols * scale) / 32) * 32; // 对齐 32 整数倍
        int target_h = (static_cast<int>(frame.rows * scale) / 32) * 32;
        target_w = std::max(32, target_w);
        target_h = std::max(32, target_h);

        cv::Mat input_frame;
        cv::resize(frame, input_frame, cv::Size(target_w, target_h));

        // 动态同步输入尺寸
        detector_->setInputSize(cv::Size(target_w, target_h));

        // 2. 记录推理耗时并执行检测
        cv::Mat faces;
        auto start_time = std::chrono::high_resolution_clock::now();
        detector_->detect(input_frame, faces);
        auto end_time = std::chrono::high_resolution_clock::now();

        std::chrono::duration<float, std::milli> duration = end_time - start_time;
        response->cost_time = duration.count();

        // 3. 解析检测结果并等比映射回原图
        int face_num = faces.rows;
        response->number = static_cast<int16_t>(face_num);

        response->left.reserve(face_num);
        response->top.reserve(face_num);
        response->right.reserve(face_num);
        response->bottom.reserve(face_num);

        float scale_x = static_cast<float>(frame.cols) / target_w;
        float scale_y = static_cast<float>(frame.rows) / target_h;

        for (int i = 0; i < face_num; ++i) {
            int32_t x = static_cast<int32_t>(faces.at<float>(i, 0) * scale_x);
            int32_t y = static_cast<int32_t>(faces.at<float>(i, 1) * scale_y);
            int32_t w = static_cast<int32_t>(faces.at<float>(i, 2) * scale_x);
            int32_t h = static_cast<int32_t>(faces.at<float>(i, 3) * scale_y);
            float score = faces.at<float>(i, 14);

            response->left.push_back(x);
            response->top.push_back(y);
            response->right.push_back(x + w);
            response->bottom.push_back(y + h);

            RCLCPP_INFO(this->get_logger(), "人脸 #%d [置信度: %.2f] 坐标: [%d, %d, %d, %d]", 
                        i + 1, score, x, y, x + w, y + h);
        }

        RCLCPP_INFO(this->get_logger(), "检测完成: 共 %d 张人脸, 推理耗时: %.2f ms", 
                    response->number, response->cost_time);
    }

public:
    FaceDetectServer(std::string name) : Node(name) {
        this->declare_parameter<std::string>(
            "model_path", 
            "/home/cds/Code/ros2/dev_ws/src/ros2_rust_tutorials/face_detection_yunet_2022mar.onnx"
        );
        this->get_parameter("model_path", model_path_);

        // 初始化 YuNet 检测器
        // 关键调整：score_threshold 设为 0.5f，确保检出侧脸与大表情
        detector_ = cv::FaceDetectorYN::create(
            model_path_,
            "",
            cv::Size(320, 320), // 初始尺寸，推理时会自动更新
            0.7f,               // 置信度阈值：0.5
            0.3f,               // NMS 抑制重叠框阈值
            5000                // TopK 候选数
        );

        if (!detector_) {
            RCLCPP_ERROR(this->get_logger(), "初始化 YuNet 模型失败: %s", model_path_.c_str());
            return;
        }

        service_ = this->create_service<learning_interface::srv::FaceDetector>(
            "face_detect",
            std::bind(&FaceDetectServer::handle_face_detect, this, 
                      std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "YuNet 人脸检测服务端已启动 (置信度阈值: 0.5)");
    }

    ~FaceDetectServer() {}
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FaceDetectServer>("face_detect_server");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}