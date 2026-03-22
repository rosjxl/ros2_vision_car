#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

class RadarNode : public rclcpp::Node {
public:
    RadarNode() : Node("TUF_radar") {
        // 架设接收天线
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", 10,
            std::bind(&RadarNode::image_callback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "天选目标雷达已上线！正在执行 HSV 色彩剥离...");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // 1. 获取彩色原图
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            
            // 2. 准备机器视觉的画布
            cv::Mat hsv_frame, mask;
            
            // 3. 将画面从人类的 BGR 强行扭曲进机器的 HSV 空间
            cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV); 
            
            // 4. 戴上“红色夜视仪” (设定红色的上下限阈值)
            cv::Scalar lower_red(0, 120, 70);
            cv::Scalar upper_red(10, 255, 255);
            
            // 5. 雷达扫描！只留下红色的物体，其余全部涂黑！
            cv::inRange(hsv_frame, lower_red, upper_red, mask); 
            
            // 6. 展现战果！同时开两个显示器
            cv::imshow("Original Camera (真实世界)", frame);
            cv::imshow("TUF Red Target Mask (高科技雷达视角)", mask);
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
