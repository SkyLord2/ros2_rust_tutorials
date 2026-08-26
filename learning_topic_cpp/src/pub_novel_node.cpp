#include <queue>
#include <chrono>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <mutex>
#include <string_view>
#include <memory>
#include <curl/curl.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;
struct CurlDeleter {
    void operator()(CURL* curl) const noexcept {
        if (curl)
        {
            curl_easy_cleanup(curl);
        }
        
    }
};

using UniqueCurl = std::unique_ptr<CURL, CurlDeleter>;

class PubNovelNode : public rclcpp::Node
{
private:
    std::string node_name;
    std::string content;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mutex queue_mutex_;
    std::thread download_thread_;
    std::queue<std::string> content_queue_;
    std::string_view url = "https://www.gutenberg.org/cache/epub/23950/pg23950.txt";
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

private:
    static size_t download_callback(void* ptr, size_t size, size_t nmemb, void* stream) noexcept {
        auto* self = static_cast<PubNovelNode*>(stream);
        size_t total_bytes = size * nmemb;
        self->content.append(static_cast<char*>(ptr), total_bytes);
        // RCLCPP_INFO(self->get_logger(), "下载内容：%s", self->content.c_str());
        size_t pos = 0;
        while ((pos = self->content.find('\n')) != std::string::npos) {
            std::string line = self->content.substr(0, pos);
            
            // 清除末尾的 Windows 回车符 \r
            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }

            // 移除缓冲区中已提取的行及 \n
            self->content.erase(0, pos + 1);

            // 3. 加锁将切好的非空行压入队列，供定时器发布
            if (!line.empty()) {
                std::lock_guard<std::mutex> lock(self->queue_mutex_);
                self->content_queue_.push(line);
            }
        }
        RCLCPP_INFO(self->get_logger(), "拆分完成，队列中共有 %zu 行小说文本", self->content_queue_.size());
        return total_bytes;
    }
    bool download_novel() {
        // 使用智能指针初始化 CURL 句柄，超出作用域后会自动调用 CurlDeleter 释放
        UniqueCurl curl(curl_easy_init());
        if (!curl) {
            RCLCPP_INFO(this->get_logger(), "CURL 句柄初始化失败！");
            return false;
        }

        // 配置 CURL 请求选项 (通过 curl.get() 获取原生 C 指针)
        curl_easy_setopt(curl.get(), CURLOPT_URL, url.data());
        curl_easy_setopt(curl.get(), CURLOPT_WRITEFUNCTION, download_callback);
        curl_easy_setopt(curl.get(), CURLOPT_WRITEDATA, this);
        curl_easy_setopt(curl.get(), CURLOPT_FOLLOWLOCATION, 1L); // 自动跟随 HTTP 301/302 重定向
        curl_easy_setopt(curl.get(), CURLOPT_TIMEOUT, 300L);        // 设置 300 秒超时时间
        curl_easy_setopt(curl.get(), CURLOPT_USERAGENT, "Mozilla/5.0 (X11; Linux x86_64)");

        RCLCPP_INFO(this->get_logger(), "启动后台流式下载小说的线程...");

        // 执行 HTTP 请求
        CURLcode res = curl_easy_perform(curl.get());
        if (res != CURLE_OK) {
            RCLCPP_INFO(this->get_logger(), "下载失败，错误描述: %s", curl_easy_strerror(res));
            return false;
        }


        // 处理文件末尾可能遗留的无 \n 行
        if (!content.empty()) {
            if (content.back() == '\r') {
                content.pop_back();
            }
            if (!content.empty()) {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                content_queue_.push(content);
            }
            content.clear();
        }

        RCLCPP_INFO(this->get_logger(), "小说下载完成，总字节数：%zu", content.size());
        return true; // 函数退出时，curl 句柄和 out_file 文件流会被自动销毁与关闭
    }
public:
    PubNovelNode(std::string name) : Node(name), node_name(name) {
        curl_global_init(CURL_GLOBAL_ALL);
        auto qos = rclcpp::QoS(10);
        publisher_ = this->create_publisher<std_msgs::msg::String>("novel_content", qos);
        timer_ = this->create_wall_timer(1s, [this]() {
            std::string line;
            // 线程安全地从队列头部取出一条数据
            {
                std::lock_guard<std::mutex> lock(this->queue_mutex_);
                if (this->content_queue_.empty()) {
                    return;
                }
                line = this->content_queue_.front();
                this->content_queue_.pop();
            }

            // 发布消息
            auto msg = std_msgs::build<std_msgs::msg::String>().data(line);
            this->publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "定时发布行（队列缓冲剩余 %zu 行）: %s", 
                        this->content_queue_.size(), line.c_str());
        });
        
        download_thread_ = std::thread(&PubNovelNode::download_novel, this);
    }
    ~PubNovelNode() {
        // 清理 libcurl 全局环境
        curl_global_cleanup();
    }
};


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PubNovelNode>("pub_novel_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
