#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp> // 新引进的子弹类型（图像消息）
#include <cv_bridge/cv_bridge.h>     // 新引进的翻译官

class CameraNode : public rclcpp::Node {
public:
    // 你的专属指挥官代号！
    CameraNode() : Node("TUF_commander") { 
        
        // 【核心改造 1】：架设一台广播发射机！频道名字叫 "camera/image_raw"
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);
        
        RCLCPP_INFO(this->get_logger(), "TUF_commander 视觉广播电台已启动！正在向全频段发送画面...");
        
        cap_.open(0);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "报告：摄像头接管失败！");
            return;
        }

        // 把你的闹钟调回 30 毫秒（大约 30 帧/秒），保证画面丝滑
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&CameraNode::timer_callback, this)
        );
    }

private:
    void timer_callback() {
        cv::Mat frame;
        cap_ >> frame; 
        if (!frame.empty()) {
            // 注意！这里不再有 cv::imshow 弹窗了！
            
            // 【核心改造 2】：炼金术！把 OpenCV 的 frame，翻译成 ROS2 的 msg
            sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            
            // 【核心改造 3】：按发射按钮！把画面狠狠地广播出去！
            publisher_->publish(*msg);
        }
    }

    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
    // 随身工具包里多了一个天线（发布者）
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}
