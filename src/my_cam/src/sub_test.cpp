#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/twist.hpp>

// 【今日终极架构：定义机器人的 FSM 状态机（潜意识）】
enum class RobotState {
    SEARCHING, // 搜寻模式：像雷达一样原地打圈
    TRACKING   // 猛扑模式：死死咬住目标并控制距离
};

class RadarNode : public rclcpp::Node {
public:
    // 初始化时，默认让它处于“搜寻模式”，清空记忆
    RadarNode() : Node("TUF_radar"), last_error_(0.0), current_state_(RobotState::SEARCHING) { 
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", 10,
            std::bind(&RadarNode::image_callback, this, std::placeholders::_1));
            
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
            
        RCLCPP_INFO(this->get_logger(), "天选目标雷达 V6.0 启动！FSM 猎手本能与 Z 轴纵深感知已接入！");
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
            auto cmd_msg = geometry_msgs::msg::Twist();
            
            // 【核心情报提取：获取目标的像素总面积（用来估算物理距离）】
            double area = M.m00; 

            // ---------------------------------------------------------
            // 脑干意识切换（状态机逻辑）：
            // 只要面积大于 500（过滤掉一点点红色的噪点），就切入“猛扑模式”！
            // 否则，强制切入“搜寻模式”！
            // ---------------------------------------------------------
            if (area > 500) {
                current_state_ = RobotState::TRACKING;
            } else {
                current_state_ = RobotState::SEARCHING;
            }

            // =========================================================
            // 状态 A：执行【猛扑模式】
            // =========================================================
            if (current_state_ == RobotState::TRACKING) {
                int cX = int(M.m10 / M.m00);
                int cY = int(M.m01 / M.m00);
                
                cv::circle(frame, cv::Point(cX, cY), 7, cv::Scalar(0, 255, 0), -1);
                
                // 1. 继承昨天的 PD 电子阻尼（解决左右画龙）
                float current_error = 320.0 - cX; 
                float d_error = current_error - last_error_; 
                float Kp = 0.005; 
                float Kd = 0.008; 
                cmd_msg.angular.z = (current_error * Kp) + (d_error * Kd); 
                last_error_ = current_error; 
                
                // 2. 【今日高地：Z 轴纵深防撞逻辑】
                // 假设安全面积在 20000 到 50000 像素之间
                if (area < 20000) {
                    cmd_msg.linear.x = 0.15; // 目标太小（太远），给正油门，往前追！
                    cv::putText(frame, "CHASING (FORWARD)", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
                } else if (area > 50000) {
                    cmd_msg.linear.x = -0.15; // 目标极大（贴脸了），给负油门，挂倒挡后退！
                    cv::putText(frame, "DANGER! (BACKWARD)", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
                } else {
                    cmd_msg.linear.x = 0.0;  // 在安全距离，停车对峙，死死盯着！
                    cv::putText(frame, "HOLD POSITION", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
                }
                
                RCLCPP_INFO(this->get_logger(), "[猛扑模式] 面积:%.0f | 油门:%.2f | 转向:%.2f", area, cmd_msg.linear.x, cmd_msg.angular.z);

            // =========================================================
            // 状态 B：执行【搜寻模式】
            // =========================================================
            } else if (current_state_ == RobotState::SEARCHING) {
                // 失去目标，绝对不能发呆！
                cmd_msg.linear.x = 0.0;   // 停止前进/后退
                cmd_msg.angular.z = 0.5;  // 赋予一个恒定的角速度，让战车原地逆时针打转！
                last_error_ = 0.0;        // 清空之前的误差记忆
                
                cv::putText(frame, "SEARCHING...", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
                RCLCPP_WARN(this->get_logger(), "[搜寻模式] 目标丢失！已切断前进动力，启动雷达自转扫描！");
            }

            // 发射终极物理指令！
            publisher_->publish(cmd_msg);
            
            cv::imshow("TUF HUD (FSM 猎手本能)", frame);
            cv::imshow("Target Mask", mask);
            cv::waitKey(1); 
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "雷达解析失败: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    float last_error_; 
    RobotState current_state_; // 【新增：记录当前战车处于什么状态】
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RadarNode>());
    rclcpp::shutdown();
    return 0;
}
