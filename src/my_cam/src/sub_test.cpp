#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/twist.hpp> // 【今日核心武器：底盘物理扭矩协议】

class RadarNode : public rclcpp::Node {
public:
    RadarNode() : Node("TUF_radar") {
        // 1. 接收天线（收画面）
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", 10,
            std::bind(&RadarNode::image_callback, this, std::placeholders::_1));
            
        // 2. 【今日新增】发射天线（发控制信号给底层车轮！）
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
            
        RCLCPP_INFO(this->get_logger(), "天选目标雷达 V4.0 启动！Twist 物理底盘控制中心已连线！");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            cv::Mat hsv_frame, mask;
            
            cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV); 
            cv::Scalar lower_red(0, 120, 70);
            cv::Scalar upper_red(10, 255, 255);
            cv::inRange(hsv_frame, lower_red, upper_red, mask); 
            
            cv::Moments M = cv::moments(mask);
            
            // 准备用来发给车轮的“速度信号空壳”
            auto cmd_msg = geometry_msgs::msg::Twist();
            
            if (M.m00 > 0) {
                int cX = int(M.m10 / M.m00);
                int cY = int(M.m01 / M.m00);
                
                cv::circle(frame, cv::Point(cX, cY), 7, cv::Scalar(0, 255, 0), -1);
                cv::putText(frame, "LOCKED", cv::Point(cX - 30, cY - 20),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                
                // 【今日终极降维打击：P 比例控制算法】
                // 假设画面宽640，中心是320。计算当前红点偏离中心的误差！
                float error = 320.0 - cX; 
                
                // 将误差乘以一个极小的比例系数 (Kp = 0.005)，转化为平滑的转向电压
                cmd_msg.angular.z = error * 0.005; 
                // 只要死死锁定目标，就给车轮下发 0.1 的恒定向前油门！
                cmd_msg.linear.x = 0.1; 
                
                RCLCPP_INFO(this->get_logger(), ">> [开火] 锁定目标 X=%d | 误差=%.1f | 动态转向电压=%.3f", cX, error, cmd_msg.angular.z);
            } else {
                // 如果画面里没有红色，立刻踩死刹车（发送 0 速度），防止战车暴走！
                cmd_msg.linear.x = 0.0;
                cmd_msg.angular.z = 0.0;
                RCLCPP_WARN(this->get_logger(), "丢失目标！已紧急切断车轮动力！");
            }
            
            // 【核心动作】：将封装好的速度和转向指令，像导弹一样打入 /cmd_vel 频道！
            publisher_->publish(cmd_msg);
            
            cv::imshow("TUF HUD (Twist 丝滑追踪视角)", frame);
            cv::imshow("Target Mask", mask);
            cv::waitKey(1); 
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "雷达解析失败: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    // 【今日新增】：底盘指令发射器
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RadarNode>());
    rclcpp::shutdown();
    return 0;
}
