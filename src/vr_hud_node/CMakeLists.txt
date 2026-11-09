cmake_minimum_required(VERSION 3.8)
project(vr_hud_node)

set(CMAKE_CXX_STANDARD 17)
set(OpenGL_GL_PREFERENCE "GLVND")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(vr_hud_src src/vr_hud_node.cpp)

target_include_directories(vr_hud_src PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(vr_hud_src ${OpenCV_LIBRARIES})

ament_target_dependencies(vr_hud_src
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

install(TARGETS vr_hud_src DESTINATION lib/${PROJECT_NAME})
ament_package()
