cmake_minimum_required(VERSION 3.8)
project(my_cam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 寻找核心组件
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# 指示工厂把 cam_test.cpp 压制成名叫 cam_node 的可执行程序
add_executable(cam_node src/cam_test.cpp)

# 强行绑定 OpenCV 视觉库和 ROS2 神经中枢
target_include_directories(cam_node PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(cam_node rclcpp OpenCV sensor_msgs cv_bridge)
target_link_libraries(cam_node ${OpenCV_LIBRARIES})

add_executable(sub_node src/sub_test.cpp)
target_include_directories(sub_node PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(sub_node rclcpp OpenCV sensor_msgs cv_bridge)
target_link_libraries(sub_node ${OpenCV_LIBRARIES})

# 将造好的程序入库到指定位置
install(TARGETS cam_node sub_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
