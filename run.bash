#!/usr/bin/env bash
rm -rf build/ install/ log/
colcon build

# spin off a new terminal
gnome-terminal --tab -- bash -lc 'ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py nav2:=true slam:=false localization:=true rviz:=true'
sleep 15

source install/setup.bash
ros2 run patrol patrol_loop
