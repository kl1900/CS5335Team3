"""Patrol code with battery monitoring, mostly inspired by

https://github.com/turtlebot/turtlebot4_tutorials/blob/jazzy/turtlebot4_python_tutorials/turtlebot4_python_tutorials/patrol_loop.py
"""

from math import floor
from threading import Lock, Thread
from time import sleep
import yaml

import rclpy

from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import BatteryState
from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Directions,
    TurtleBot4Navigator,
)


_DIRECTION_MAP = {
    "NORTH": TurtleBot4Directions.NORTH,
    "SOUTH": TurtleBot4Directions.SOUTH,
    "EAST": TurtleBot4Directions.EAST,
    "WEST": TurtleBot4Directions.WEST,
    "NORTH_EAST": TurtleBot4Directions.NORTH_EAST,
    "NORTH_WEST": TurtleBot4Directions.NORTH_WEST,
    "SOUTH_EAST": TurtleBot4Directions.SOUTH_EAST,
    "SOUTH_WEST": TurtleBot4Directions.SOUTH_WEST,
}


class BatteryMonitor(Node):
    def __init__(self, lock):
        super().__init__("battery_monitor")

        self.lock = lock

        # Subscribe to the /battery_state topic
        self.battery_state_subscriber = self.create_subscription(
            BatteryState,
            "battery_state",
            self.battery_state_callback,
            qos_profile_sensor_data,
        )

    # Callbacks
    def battery_state_callback(self, batt_msg: BatteryState):
        with self.lock:
            self.battery_percent = batt_msg.percentage

    def thread_function(self):
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        executor.spin()


def load_navigation_config(yaml_path: str, navigator) -> dict:
    """
    Load battery thresholds and goal poses from a YAML file.

    Expected YAML format:
    battery_high: 0.80
    battery_low: 0.20
    battery_critical: 0.10

    goals:
      - x: -5.0
        y: 1.0
        direction: EAST
      - x: -5.0
        y: -23.0
        direction: NORTH

    Returns:
        {
            "battery_high": float,
            "battery_low": float,
            "battery_critical": float,
            "goal_poses": list,
        }
    """
    with open(yaml_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict):
        raise ValueError("YAML root must be a dictionary.")

    # Load battery thresholds
    required_battery_keys = ["battery_high", "battery_low", "battery_critical"]
    for key in required_battery_keys:
        if key not in data:
            raise ValueError(f"Missing required battery config key: '{key}'")

    battery_high = float(data["battery_high"])
    battery_low = float(data["battery_low"])
    battery_critical = float(data["battery_critical"])

    # Optional sanity checks
    if not (0.0 <= battery_critical <= 1.0):
        raise ValueError("battery_critical must be between 0.0 and 1.0")
    if not (0.0 <= battery_low <= 1.0):
        raise ValueError("battery_low must be between 0.0 and 1.0")
    if not (0.0 <= battery_high <= 1.0):
        raise ValueError("battery_high must be between 0.0 and 1.0")
    if not (battery_critical <= battery_low <= battery_high):
        raise ValueError(
            "Battery thresholds must satisfy: "
            "battery_critical <= battery_low <= battery_high"
        )

    # Load goals
    if "goals" not in data:
        raise ValueError("YAML must contain a top-level 'goals' list.")

    goals = data["goals"]
    if not isinstance(goals, list):
        raise ValueError("'goals' must be a list.")

    goal_poses = []

    for i, goal in enumerate(goals):
        if not isinstance(goal, dict):
            raise ValueError(f"Goal at index {i} must be a dictionary.")

        try:
            x = float(goal["x"])
            y = float(goal["y"])
            direction_str = str(goal["direction"]).upper()
        except KeyError as e:
            raise ValueError(f"Goal at index {i} is missing required field: {e}")

        if direction_str not in _DIRECTION_MAP:
            valid = ", ".join(_DIRECTION_MAP.keys())
            raise ValueError(
                f"Invalid direction '{direction_str}' at goal index {i}. "
                f"Valid directions: {valid}"
            )

        direction = _DIRECTION_MAP[direction_str]
        goal_poses.append(navigator.getPoseStamped([x, y], direction))

    return {
        "battery_high": battery_high,
        "battery_low": battery_low,
        "battery_critical": battery_critical,
        "goal_poses": goal_poses,
    }


def main(args=None):
    rclpy.init(args=args)
    lock = Lock()
    battery_monitor = BatteryMonitor(lock)
    # Temporary node only for reading ROS parameters
    param_node = Node("navigator_config_loader")
    param_node.declare_parameter("config", "./params.yml")
    config_path = (
        param_node.get_parameter("config").get_parameter_value().string_value
    )
    param_node.get_logger().info(f"Using config file: {config_path}")
    param_node.destroy_node()

    navigator = TurtleBot4Navigator()
    battery_percent = None
    position_index = 0

    thread = Thread(target=battery_monitor.thread_function, daemon=True)
    thread.start()

    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info("Docking before intialising pose")
        navigator.dock()

    # Set initial pose
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Undock
    navigator.undock()

    # configure goals and battery
    config = load_navigation_config(config_path, navigator)
    battery_high = config["battery_high"]
    battery_low = config["battery_low"]
    battery_critical = config["battery_critical"]
    goal_pose = config["goal_poses"]
    navigator.info(
        "Parsed config:\n"
        f"  battery_high: {battery_high:.2f}\n"
        f"  battery_low: {battery_low:.2f}\n"
        f"  battery_critical: {battery_critical:.2f}\n"
        f"  goal_count: {len(goal_pose)}"
    )

    while True:
        with lock:
            battery_percent = battery_monitor.battery_percent

        if battery_percent is not None:
            navigator.info(f"Battery is at {(battery_percent*100):.2f}% charge")

            # Check battery charge level
            if battery_percent < battery_critical:
                navigator.error("Battery critically low. Charge or power down")
                break
            elif battery_percent < battery_low:
                # Go near the dock
                navigator.info("Docking for charge")
                navigator.startToPose(
                    navigator.getPoseStamped([-1.0, 1.0], TurtleBot4Directions.EAST)
                )
                navigator.dock()

                if not navigator.getDockedStatus():
                    navigator.error("Robot failed to dock")
                    break

                # Wait until charged
                navigator.info("Charging...")
                battery_percent_prev = 0
                while battery_percent < battery_high:
                    sleep(15)
                    battery_percent_prev = floor(battery_percent * 100) / 100
                    with lock:
                        battery_percent = battery_monitor.battery_percent

                    # Print charge level every time it increases a percent
                    if battery_percent > (battery_percent_prev + 0.01):
                        navigator.info(
                            f"Battery is at {(battery_percent*100):.2f}% charge"
                        )

                # Undock
                navigator.undock()
                position_index = 0

            else:
                # Navigate to next position
                navigator.startToPose(goal_pose[position_index])

                position_index = position_index + 1
                if position_index >= len(goal_pose):
                    position_index = 0

    battery_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
