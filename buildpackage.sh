cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select vr_hud_node
source ~/ros2_ws/install/setup.bash
ros2 run vr_hud_node vr_hud_src
