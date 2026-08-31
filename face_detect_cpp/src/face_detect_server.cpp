#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <learning_interface/srv/face_detector.hpp>

class FaceDetectServer : public rclcpp::Node
{
private:
    rclcpp::Service<learning_interface::srv::FaceDetector>::SharedPtr service_;
    cv::Ptr<cv::FaceDetectorYN> detector_; // YuNet 人脸检测器实例
    std::string model_path_;

    // 推荐固定推理尺寸（兼顾微小人脸检测精度与超低推理延迟）
    const cv::Size INFERENCE_SIZE{640, 640};

private:
    void handle_face_detect(
        const std::shared_ptr<learning_interface::srv::FaceDetector::Request> request,
        std::shared_ptr<learning_interface::srv::FaceDetector::Response> response
    ) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            // 1. 将 ROS 图像消息转换为 OpenCV 的 BGR8 格式 cv::Mat
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

        // 2. 将图像缩放到固定的推理尺寸，保证 DNN 网络的稳定性与速度
        cv::Mat input_frame;
        cv::resize(frame, input_frame, INFERENCE_SIZE);

        // 3. 记录检测耗时并执行推理
        cv::Mat faces;
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // 传入 640x640 图像进行检测
        detector_->detect(input_frame, faces);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float, std::milli> duration = end_time - start_time;
        response->cost_time = duration.count(); // 写入毫秒级耗时

        // 4. 解析人脸检测结果矩阵
        int face_num = faces.rows; // 检测到的总人脸数
        response->number = static_cast<int16_t>(face_num);

        // 预分配向量内存空间
        response->left.reserve(face_num);
        response->top.reserve(face_num);
        response->right.reserve(face_num);
        response->bottom.reserve(face_num);

        // 计算原图与推理图像的缩放比例
        float scale_x = static_cast<float>(frame.cols) / INFERENCE_SIZE.width;
        float scale_y = static_cast<float>(frame.rows) / INFERENCE_SIZE.height;

        for (int i = 0; i < face_num; ++i) {
            // 将推理结果坐标等比例映射回原图坐标
            int32_t x = static_cast<int32_t>(faces.at<float>(i, 0) * scale_x);
            int32_t y = static_cast<int32_t>(faces.at<float>(i, 1) * scale_y);
            int32_t w = static_cast<int32_t>(faces.at<float>(i, 2) * scale_x);
            int32_t h = static_cast<int32_t>(faces.at<float>(i, 3) * scale_y);
            float score = faces.at<float>(i, 14); // 提取置信度评分

            // 存入边界框坐标
            response->left.push_back(x);
            response->top.push_back(y);
            response->right.push_back(x + w);
            response->bottom.push_back(y + h);

            RCLCPP_INFO(this->get_logger(), "人脸 #%d 置信度: %.2f, 坐标: [%d, %d, %d, %d]", 
                        i + 1, score, x, y, x + w, y + h);
        }

        RCLCPP_INFO(this->get_logger(), "检测完成: 共 %d 张人脸, 推理耗时: %.2f ms", 
                    response->number, response->cost_time);
    }

public:
    FaceDetectServer(std::string name) : Node(name) {
        // 声明与获取 ONNX 模型路径参数，默认使用 2022mar 模型
        this->declare_parameter<std::string>(
            "model_path", 
            "/home/cds/Code/ros2/dev_ws/src/ros2_rust_tutorials/face_detection_yunet_2022mar.onnx"
        );
        this->get_parameter("model_path", model_path_);

        // 初始化 YuNet 检测器 (固定输入尺寸为 640x640)
        detector_ = cv::FaceDetectorYN::create(
            model_path_,
            "",
            INFERENCE_SIZE,     // 与推理尺寸保持一致
            0.8f,               // 置信度阈值 (score threshold)
            0.3f,               // 非极大值抑制阈值 (nms threshold)
            5000                // 最大保留人脸候选数
        );

        if (!detector_) {
            RCLCPP_ERROR(this->get_logger(), "初始化 YuNet 模型失败，请检查模型路径: %s", model_path_.c_str());
            return;
        }

        // 创建服务并绑定回调函数
        service_ = this->create_service<learning_interface::srv::FaceDetector>(
            "face_detect",
            std::bind(&FaceDetectServer::handle_face_detect, this, 
                      std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "YuNet 人脸检测服务端已启动，模型加载路径: %s", model_path_.c_str());
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