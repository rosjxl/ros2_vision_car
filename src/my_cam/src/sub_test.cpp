#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

class RadarNode : public rclcpp::Node {
public:
    RadarNode() : Node("TUF_radar") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", 10,
            std::bind(&RadarNode::image_callback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "天选目标雷达 V2.0 启动！正在执行几何降维与坐标锁定...");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            cv::Mat hsv_frame, mask;
            
            // 1. 强行扭曲进 HSV 空间并戴上红色夜视仪
            cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV); 
            cv::Scalar lower_red(0, 120, 70);
            cv::Scalar upper_red(10, 255, 255);
            cv::inRange(hsv_frame, lower_red, upper_red, mask); 
            
            // 【第二阶段战术突入：图像矩算法锁定物理重心】
            cv::Moments M = cv::moments(mask);
            
            // 防止除以零（如果屏幕上根本没有红色，程序会崩溃，必须加护盾！）
            if (M.m00 > 0) {
                // 核心数学降维：计算重心的 X 和 Y 坐标
                int cX = int(M.m10 / M.m00);
                int cY = int(M.m01 / M.m00);
                
                // 战术反馈 A：在彩色原图上画一个绿色的十字准星！
                cv::circle(frame, cv::Point(cX, cY), 7, cv::Scalar(0, 255, 0), -1);
                cv::putText(frame, "LOCKED", cv::Point(cX - 30, cY - 20),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                
                // 战术反馈 B：在终端里实时打印狙击坐标！
                RCLCPP_INFO(this->get_logger(), ">> 目标锁定！当前坐标: X=%d, Y=%d", cX, cY);
            } else {
                RCLCPP_WARN(this->get_logger(), "丢失目标！全屏搜索中...");
            }
            
            // 展现战果！
            cv::imshow("TUF HUD (真实世界+准星)", frame);
            cv::imshow("Target Mask (高科技雷达)", mask);
            cv::waitKey(1); 
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "报告：雷达解析失败！错误: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RadarNode>());
    rclcpp::shutdown();
    return 0;
}
