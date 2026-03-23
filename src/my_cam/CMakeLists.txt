cmake_minimum_required(VERSION 3.8)
project(my_cam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 寻找核心物资
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
# 【今日新增】：寻找底盘控制协议
find_package(geometry_msgs REQUIRED)

# 2. 组装一号兵种：电台 (cam_node) -> 昨天的心血，完整保留！
add_executable(cam_node src/cam_test.cpp)
ament_target_dependencies(cam_node rclcpp sensor_msgs cv_bridge OpenCV)

# 3. 组装二号兵种：雷达与决策中枢 (sub_node)
add_executable(sub_node src/sub_test.cpp)
# 【今日新增】：给雷达兵装配 geometry_msgs 武器库！
ament_target_dependencies(sub_node rclcpp sensor_msgs cv_bridge OpenCV geometry_msgs)

# 4. 统一下发安装指令
install(TARGETS
  cam_node
  sub_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
