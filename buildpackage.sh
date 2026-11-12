cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select vr_hud_node --cmake-args -DOpenCV_DIR=/usr/local/lib/cmake/opencv5
source ~/ros2_ws/install/setup.bash
ros2 launch vr_hud_node vr_hud.launch.py
