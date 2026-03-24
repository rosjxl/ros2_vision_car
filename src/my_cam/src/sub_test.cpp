#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp> // 【今日终极隐藏武器：ArUco 工业视觉库】
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/twist.hpp>

enum class RobotState {
    SEARCHING, 
    TRACKING   
};

class RadarNode : public rclcpp::Node {
public:
    RadarNode() : Node("TUF_radar"), last_error_(0.0), current_state_(RobotState::SEARCHING) { 
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", 10,
            std::bind(&RadarNode::image_callback, this, std::placeholders::_1));
            
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
            
        RCLCPP_INFO(this->get_logger(), "天选目标雷达 V7.0 终极版启动！ArUco 工业二维码接管视觉中枢！");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            auto cmd_msg = geometry_msgs::msg::Twist();

            // 1. 调取工业标准 DICT_6X6_250 字典（包含250种不同的二维码）
            cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
            cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
            
            // 2. 释放 ArUco 扫描射线！
            cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

            // =========================================================
            // 脑干意识切换：看到任何一个二维码，立刻切入猛扑模式！
            // =========================================================
            if (markerIds.size() > 0) {
                current_state_ = RobotState::TRACKING;
            } else {
                current_state_ = RobotState::SEARCHING;
            }

            if (current_state_ == RobotState::TRACKING) {
                // 在画面上用绿线框出工业二维码！
                cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
                
                // 算出二维码中心点 (X, Y)
                float cX = (markerCorners[0][0].x + markerCorners[0][1].x + markerCorners[0][2].x + markerCorners[0][3].x) / 4.0;
                float cY = (markerCorners[0][0].y + markerCorners[0][1].y + markerCorners[0][2].y + markerCorners[0][3].y) / 4.0;
                
                // 算出二维码在画面里的近似面积（防撞测距用）
                float width = cv::norm(markerCorners[0][0] - markerCorners[0][1]);
                float height = cv::norm(markerCorners[0][1] - markerCorners[0][2]);
                float area = width * height;

                // 3. PD 电子阻尼转向控制
                float current_error = 320.0 - cX; 
                float d_error = current_error - last_error_; 
                float Kp = 0.005; 
                float Kd = 0.008; 
                cmd_msg.angular.z = (current_error * Kp) + (d_error * Kd); 
                last_error_ = current_error; 
                
                // 4. Z 轴纵深防撞（二维码面积）
                if (area < 15000) {
                    cmd_msg.linear.x = 0.15; // 太远，追击！
                    cv::putText(frame, "TRACKING ID:" + std::to_string(markerIds[0]) + " (FORWARD)", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
                } else if (area > 40000) {
                    cmd_msg.linear.x = -0.15; // 贴脸，倒车！
                    cv::putText(frame, "DANGER! (BACKWARD)", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
                } else {
                    cmd_msg.linear.x = 0.0;  // 停车对峙
                    cv::putText(frame, "HOLD POSITION", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
                }
                
                RCLCPP_INFO(this->get_logger(), "锁定 ID:%d | 面积:%.0f | 转向电压:%.3f", markerIds[0], area, cmd_msg.angular.z);

            } else if (current_state_ == RobotState::SEARCHING) {
                // 失去目标，雷达自转扫街！
                cmd_msg.linear.x = 0.0;   
                cmd_msg.angular.z = 0.5;  
                last_error_ = 0.0;        
                
                cv::putText(frame, "SEARCHING ArUco...", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
            }

            publisher_->publish(cmd_msg);
            cv::imshow("TUF HUD (ArUco Industrial View)", frame);
            cv::waitKey(1); 
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "雷达解析失败: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    float last_error_; 
    RobotState current_state_; 
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RadarNode>());
    rclcpp::shutdown();
    return 0;
}
