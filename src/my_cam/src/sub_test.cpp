#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/twist.hpp> 

class RadarNode : public rclcpp::Node {
public:
    RadarNode() : Node("TUF_radar"), last_error_(0.0) { // 【注意这里：初始化记忆中枢，上一帧误差设为0】
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", 10,
            std::bind(&RadarNode::image_callback, this, std::placeholders::_1));
            
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
            
        RCLCPP_INFO(this->get_logger(), "天选目标雷达 V5.0 终极形态启动！PD 电子阻尼器已接入！");
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
            
            if (M.m00 > 0) {
                int cX = int(M.m10 / M.m00);
                int cY = int(M.m01 / M.m00);
                
                cv::circle(frame, cv::Point(cX, cY), 7, cv::Scalar(0, 255, 0), -1);
                cv::putText(frame, "LOCKED", cv::Point(cX - 30, cY - 20),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                
                // 【终极降维打击：PD 控制算法】
                float current_error = 320.0 - cX; // 1. 算出现在偏了多少
                float d_error = current_error - last_error_; // 2. 算出现在移动得多快（微积分预判！）
                
                // 极客调参备忘录：Kp 是油门，Kd 是刹车。这两个数字在真车上需要你反复测试修改！
                float Kp = 0.005; 
                float Kd = 0.008; 
                
                // 3. 终极转向电压 = (P控制油门) + (D控制电子刹车)
                cmd_msg.angular.z = (current_error * Kp) + (d_error * Kd); 
                cmd_msg.linear.x = 0.1; // 保持恒定往前开
                
                // 4. 将现在的误差刻进记忆里，留给下一帧用作对比！
                last_error_ = current_error; 
                
                RCLCPP_INFO(this->get_logger(), ">> 目标锁定！当前误差: %.1f | D阻尼: %.1f | 综合输出电压: %.3f", current_error, d_error, cmd_msg.angular.z);
            } else {
                cmd_msg.linear.x = 0.0;
                cmd_msg.angular.z = 0.0;
                last_error_ = 0.0; // 丢失目标时，清空记忆
                RCLCPP_WARN(this->get_logger(), "丢失目标！紧急刹车断电！");
            }
            
            publisher_->publish(cmd_msg);
            
            cv::imshow("TUF HUD (PD 丝滑追踪)", frame);
            cv::imshow("Target Mask", mask);
            cv::waitKey(1); 
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "雷达解析失败: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    
    // 【今日核心新增：雷达的短时记忆中枢，用来存储上一帧的误差】
    float last_error_; 
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RadarNode>());
    rclcpp::shutdown();
    return 0;
}
