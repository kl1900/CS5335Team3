# CS5335 Team 3 Project

[Proposal Link](https://docs.google.com/document/d/1rNEDmhcds6Ypxy9_O8n1p-JhhkwlK8lQpenq4bSY6Y8/edit?usp=sharing)

[Powerpoint Link](https://docs.google.com/presentation/d/1fs7SOLTvtHNyI32KZREAG7GUd3kFVpcYzrHSkBopCqQ/edit?slide=id.p9#slide=id.p9)

[Simulation Video Link](https://drive.google.com/file/d/1MK8fB7xf5hbMFfOe3o7p1UrOKtWTYU6Z/view?usp=sharing)

[Turtlebot Robot video Link](https://drive.google.com/file/d/1dghiQFLf2fhBGTypsvCxqyDWE4qwN9BK/view?usp=sharing)
# Build and run
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
to run simulation
```bash
# terminal 1
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py nav2:=true slam:=false localization:=true rviz:=true
# terminal 2
ros2 run patrol patrol_loop
```

To run with Turtlebot 4
With our specific setting
```bash
# Terminal 1
bashsource /opt/ros/humble/setup.bash
export ROS_DISCOVERY_SERVER="192.168.50.31:11811;"
ros2 launch turtlebot4_navigation localization.launch.py map:=/<path>/<to>/<map>.yaml namespace:=/turtlebot468

# Terminal 2 — Nav2
bashsource /opt/ros/humble/setup.bash
export ROS_DISCOVERY_SERVER="192.168.50.31:11811;"
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/turtlebot468

# Terminal 3 — Patrol loop (reads params.yml automatically)
bashsource /opt/ros/humble/setup.bash
export ROS_DISCOVERY_SERVER="192.168.50.31:11811;"
source install/setup.bash
ros2 run patrol patrol_loop --ros-args -r __ns:=/turtlebot468 -p config:=/<path>/<to>/params.yml
```

# Parameters
you can edit patrol way points as well as battery thresholds in [params.yml](./params.yml). For instance
```yaml
battery_high: 0.99
battery_low: 0.98
battery_critical: 0.10

goals:
  - x: -5.0
    y: 1.0
    direction: EAST

  - x: -5.0
    y: -23.0
    direction: NORTH

  - x: 9.0
    y: -23.0
    direction: NORTH_WEST

  - x: 10.0
    y: 2.0
    direction: WEST

```

