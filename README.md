# CS5335 Team 3 Project

[Proposal Link](https://docs.google.com/document/d/1rNEDmhcds6Ypxy9_O8n1p-JhhkwlK8lQpenq4bSY6Y8/edit?usp=sharing)

[Simulation Video Link](https://drive.google.com/file/d/1MK8fB7xf5hbMFfOe3o7p1UrOKtWTYU6Z/view?usp=sharing)

[Turtlebot Robot video (TBD)]()
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
to run simulation
```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py nav2:=true slam:=false localization:=true rviz:=true

ros2 run patrol patrol_loop
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

