#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

class RadarNode : public rclcpp::Node {
public:
    RadarNode() : Node("TUF_radar") {
        // 【核心改造 1】：架设接收天线，死死盯住昨天那个 "camera/image_raw" 频道！
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", 10,
            std::bind(&RadarNode::image_callback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "天选雷达已上线！正在拦截画面并注入 OpenCV 算法...");
    }

private:
    // 这就是雷达的“工作手册”：一旦频道里有画面传来，立刻触发这个函数！
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // 【核心改造 2】：逆向翻译！把 ROS2 电波变回 OpenCV 认识的矩阵
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            
            // 【核心改造 3】：机器视觉降维打击！(边缘提取算法)
            cv::Mat gray_frame, edges;
            // 动作A：抽干色彩，变成纯灰度图 (计算量骤减)
            cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY); 
            // 动作B：提取轮廓边缘 (未来用于找车道线)
            cv::Canny(gray_frame, edges, 50, 150); 
            
            // 展现战果！
            cv::imshow("TUF Intercept & Canny Edge Vision", edges);
            cv::waitKey(1); // 刷新屏幕
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "报告：雷达解析失败！错误: %s", e.what());
        }
    }

    // 随身工具包：一根订阅者天线
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RadarNode>());
    rclcpp::shutdown();
    return 0;
}
