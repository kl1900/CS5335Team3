# CS5335 Team 3 Project

[Proposal Link](https://docs.google.com/document/d/1rNEDmhcds6Ypxy9_O8n1p-JhhkwlK8lQpenq4bSY6Y8/edit?usp=sharing)

# Build
```
cd <path>/<to>/<project>
```
clean up previous build if necessary
```
rm -rf build install log
```
then run
```
colcon build
source install/setup.bash
```
## Building each package individually
For development purpose, to build each package individually, run
```
colcon build --packages-select <my_robot_package>
source install/setup.bash
```
to clean individual package, run
```
rm -rf build/<my_robot_package> install/<my_robot_package> log
```
You can existing built packages via
```
colcon list
```

# Run
```
source install/setup.bash
ros2 run <my_robot_package> <entry_point>
```
where `entry_point` is set in `setup.py` file

# Test
To test packages run, after adding tests, run
```
colcon test --packages-select <my_robot_package>
colcon test-result --verbose
```

# commands
```
1.
source /opt/ros/humble/setup.bash
export ROS_DISCOVERY_SERVER="192.168.50.31:11811;"
ros2 launch turtlebot4_navigation localization.launch.py map:=/home/team3/CS5335Team3-1/docs/robotics_lab_map.yaml namespace:=/turtlebot468

2.
source /opt/ros/humble/setup.bash
export ROS_DISCOVERY_SERVER="192.168.50.31:11811;"
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/turtlebot468

3.
source /opt/ros/humble/setup.bash
export ROS_DISCOVERY_SERVER="192.168.50.31:11811;"
cd /home/team3/CS5335Team3-1
colcon build --packages-select battery_monitor
source install/setup.bash
ros2 run battery_monitor battery_monitor --ros-args -r __ns:=/turtlebot468

4.
source /opt/ros/humble/setup.bash
export ROS_DISCOVERY_SERVER="192.168.50.31:11811;"
cd /home/team3/CS5335Team3-1
source install/setup.bash
ros2 run battery_monitor patrol_with_battery --ros-args -r __ns:=/turtlebot468
```
