cmake_minimum_required(VERSION 3.8)
project(vr_hud_node)

set(CMAKE_CXX_STANDARD 17)
set(OpenGL_GL_PREFERENCE "GLVND")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(vr_hud_src src/vr_hud_node.cpp)

# Включаем CUDA-путь, только если OpenCV собран с cudawarping
if("opencv_cudawarping" IN_LIST OpenCV_LIBS)
  message(STATUS "OpenCV with CUDA found: building GPU path")
  target_compile_definitions(vr_hud_src PRIVATE VR_HUD_USE_CUDA)
else()
  message(STATUS "OpenCV without CUDA: building CPU path only")
endif()

target_include_directories(vr_hud_src PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(vr_hud_src ${OpenCV_LIBS})

ament_target_dependencies(vr_hud_src
  rclcpp
  sensor_msgs
  std_msgs
)

add_executable(joy_handler src/joy_handler.cpp)
ament_target_dependencies(joy_handler rclcpp sensor_msgs std_msgs)

install(TARGETS
  vr_hud_src
  joy_handler
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()