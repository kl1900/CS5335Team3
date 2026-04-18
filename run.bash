rm -rf build/ install/ log/
colcon build
source install/setup.bash
ros2 run patrol patrol_loop
