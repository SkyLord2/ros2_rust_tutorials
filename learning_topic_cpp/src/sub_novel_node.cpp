#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <atomic>

class SubNovelNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::queue<std::string> content_queue_;
    std::thread speaking_thread_;
    std::mutex queue_mutex_;
    std::condition_variable cv_;
    std::atomic<bool> is_running_{true};
private:

public:
    SubNovelNode(std::string name) : Node(name) {
        auto qos = rclcpp::QoS(10);
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "novel_content", 
            qos, 
            [this](const std_msgs::msg::String::SharedPtr msg) {
                {
                    std::lock_guard<std::mutex> lock(queue_mutex_);
                    content_queue_.push(msg->data);
                }
                cv_.notify_one();
            }
        );
        speaking_thread_ = std::thread([this]() {
            while (is_running_ && rclcpp::ok())
            {
                std::string content;
                {
                    std::unique_lock<std::mutex> lock(queue_mutex_);
                    cv_.wait(lock, [this]() {
                        return !content_queue_.empty() || !is_running_ || !rclcpp::ok();
                    });

                    if (!is_running_ || !rclcpp::ok())
                    {
                        break;
                    }
                    

                    if (!content_queue_.empty())
                    {
                        content = content_queue_.front();
                        content_queue_.pop();    
                    }
                }
                
                if (!content.empty())
                {
                    RCLCPP_INFO(this->get_logger(), "小说内容：%s", content.c_str());
                    // todo 实现语音播放
                }
                
            }
        });
    }
    ~SubNovelNode() {
        is_running_ = false;
        cv_.notify_all();
        if (speaking_thread_.joinable())
        {
            speaking_thread_.join();
        }
        
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SubNovelNode>("sub_novel_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
