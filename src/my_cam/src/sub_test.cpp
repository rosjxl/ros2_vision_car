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
            
        RCLCPP_INFO(this->get_logger(), "天选目标雷达 V3.0 终极形态启动！脑干控制中枢已连线！");
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
            
            // 2. 图像矩算法锁定物理重心
            cv::Moments M = cv::moments(mask);
            
            // 确保屏幕上有红色目标，防止程序崩溃
            if (M.m00 > 0) {
                int cX = int(M.m10 / M.m00);
                int cY = int(M.m01 / M.m00);
                
                // 画出绿色十字准星
                cv::circle(frame, cv::Point(cX, cY), 7, cv::Scalar(0, 255, 0), -1);
                cv::putText(frame, "LOCKED", cv::Point(cX - 30, cY - 20),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                
                // 【第三战区终极突入：脑干决策闭环！】
                // 画面宽度通常为640，中心安全瞄准区设定为 280 ~ 360
                if (cX < 280) {
                    RCLCPP_WARN(this->get_logger(), ">> [指令下达]：目标偏左 (X=%d)！战车左满舵转弯！ <<<", cX);
                } else if (cX > 360) {
                    RCLCPP_WARN(this->get_logger(), ">> [指令下达]：目标偏右 (X=%d)！战车右满舵转弯！ >>>", cX);
                } else {
                    RCLCPP_INFO(this->get_logger(), ">> [指令下达]：已锁定正前方 (X=%d)！战车全速直行！ ^^^", cX);
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "丢失目标！全屏搜索中...");
            }
            
            cv::imshow("TUF HUD (终极指挥视角)", frame);
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
