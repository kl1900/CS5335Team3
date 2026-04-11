#!/usr/bin/env python3
"""
Patrol node that follows Peter's path and obeys battery commands
published by battery_monitor.py on the /battery_command topic.

Commands from battery_monitor:
  'resume' → undock and begin/continue patrolling
  'dock'   → abort current leg, follow return path, dock

Patrol loops indefinitely until the node is killed.

Fix for Issue 7:
  Emergency docking (dock command received during return leg) no longer calls
  dock() immediately from wherever the robot is. Instead it first navigates to
  a known pre-dock pose close to the dock station, then calls dock(). This
  gives the TurtleBot4 auto-dock a reliable starting position to work from.
"""

import time
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import TaskResult
from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Directions,
    TurtleBot4Navigator,
)

# Pre-dock pose: last waypoint of Peter's return path, close to the dock station.
# The robot navigates here before calling dock() in emergency situations.
PRE_DOCK_POSE = [-0.0992, 0.0579]


class PatrolWithBatteryNode(Node):
    """Patrol node driven by commands from BatteryMonitorNode."""

    def __init__(self):
        super().__init__('patrol_with_battery_node')

        # ── Navigator (Peter's setup) ─────────────────────────────────────
        self.navigator = TurtleBot4Navigator()

        # ── State flags set by /battery_command callbacks ─────────────────
        self.is_docked: bool = True
        self.dock_command_received: bool = False
        self.resume_command_received: bool = False

        # ── Subscribe to battery_monitor's commands ───────────────────────
        self.create_subscription(
            String,
            '/battery_command',
            self._battery_command_callback,
            10,
        )

        # ── Status publisher ──────────────────────────────────────────────
        self.status_pub = self.create_publisher(String, '/patrol_status', 10)

        self.get_logger().info('PatrolWithBatteryNode initialised.')

    # ── Battery command callback ──────────────────────────────────────────────

    def _battery_command_callback(self, msg: String) -> None:
        """
        Receive commands from battery_monitor.py on /battery_command.
          'dock'   → flag that we need to abort current leg and return to dock.
          'resume' → flag that battery is high enough to leave dock and patrol.
        """
        command = msg.data
        self.get_logger().info(f'Received battery command: {command}')
        if command == 'dock':
            self.dock_command_received = True
        elif command == 'resume':
            self.resume_command_received = True

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(f'[status] {text}')

    def _spin(self) -> None:
        """Process one round of pending ROS callbacks."""
        rclpy.spin_once(self, timeout_sec=0.1)

    # ── Nav helpers ───────────────────────────────────────────────────────────

    def _go_to_pose(self, pose) -> TaskResult:
        """Navigate to a single pose, blocking until complete or dock commanded."""
        self.navigator.startToPose(pose)
        while not self.navigator.isTaskComplete():
            self._spin()
            if self.dock_command_received:
                self.navigator.cancelTask()
                break
            time.sleep(0.5)
        return self.navigator.getResult()

    def _navigate_waypoints(self, waypoints: list, label: str) -> bool:
        """
        Navigate through a list of waypoints one by one.
        Returns False early if a 'dock' command arrives from battery_monitor.
        """
        for i, wp in enumerate(waypoints):
            self._spin()

            if self.dock_command_received:
                self.get_logger().warn(
                    f'Dock command received before waypoint {i + 1}/{len(waypoints)} '
                    f'of {label}. Aborting leg.'
                )
                return False

            self.get_logger().info(
                f'[{label}] Navigating to waypoint {i + 1}/{len(waypoints)}…'
            )
            result = self._go_to_pose(wp)

            if self.dock_command_received:
                self.get_logger().warn(
                    f'Dock command received during waypoint {i + 1}. Aborting leg.'
                )
                return False

            if result != TaskResult.SUCCEEDED:
                self.get_logger().warn(f'Waypoint {i + 1} failed, continuing…')
            else:
                self.get_logger().info(f'Waypoint {i + 1} succeeded.')

        return True

    # ── Peter's path ──────────────────────────────────────────────────────────

    def _build_outbound_waypoints(self) -> list:
        return [
            self.navigator.getPoseStamped([-2.29, -1.47], TurtleBot4Directions.NORTH),
            self.navigator.getPoseStamped([-2.46, -1.8],  TurtleBot4Directions.NORTH),
            self.navigator.getPoseStamped([-1.94, -3.81], TurtleBot4Directions.NORTH),
            self.navigator.getPoseStamped([2.78,  -4.69], TurtleBot4Directions.NORTH),
            self.navigator.getPoseStamped([2.58,  -1.83], TurtleBot4Directions.NORTH),
        ]

    def _build_return_waypoints(self) -> list:
        return [
            self.navigator.getPoseStamped([2.78,  -4.69], TurtleBot4Directions.NORTH),
            self.navigator.getPoseStamped([-1.94, -3.81], TurtleBot4Directions.NORTH),
            self.navigator.getPoseStamped([-2.46, -1.8],  TurtleBot4Directions.NORTH),
            self.navigator.getPoseStamped([-2.29, -1.47], TurtleBot4Directions.NORTH),
            self.navigator.getPoseStamped([-0.0992, 0.0579], TurtleBot4Directions.NORTH),
        ]

    # ── Initialisation (Peter's pose setup) ──────────────────────────────────

    def _initialise_pose(self) -> None:
        initial_pose = self.navigator.getPoseStamped(
            [0.266, 0.175], TurtleBot4Directions.NORTH
        )
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.navigator.initial_pose = initial_pose
        self.navigator.initial_pose_received = False

        msg = PoseWithCovarianceStamped()
        msg.pose.pose = initial_pose.pose
        msg.header.frame_id = 'map'
        msg.header.stamp = self.navigator.get_clock().now().to_msg()
        self.navigator.initial_pose_pub.publish(msg)
        self.navigator.info('Published initial pose with fresh timestamp.')

        time.sleep(2.0)
        rclpy.spin_once(self.navigator, timeout_sec=2.0)
        self.navigator.initial_pose_received = True

    # ── Dock / charge-wait helpers ────────────────────────────────────────────

    def _navigate_to_pre_dock_and_dock(self, status_label: str) -> None:
        """
        Navigate to the pre-dock pose first, then call dock().
        Used for both normal and emergency docking to ensure the robot
        is close enough and aligned for the auto-dock to succeed.

        Fix for Issue 7: replaces the old immediate dock() call in the
        emergency case, where the robot could be anywhere on the map.
        """
        self.get_logger().info('Navigating to pre-dock pose before docking…')
        pre_dock = self.navigator.getPoseStamped(
            PRE_DOCK_POSE, TurtleBot4Directions.NORTH
        )
        result = self._go_to_pose(pre_dock)
        if result != TaskResult.SUCCEEDED:
            self.get_logger().warn(
                'Could not reach pre-dock pose — attempting dock anyway.'
            )
        self.get_logger().info('Docking…')
        self.navigator.dock()
        self.is_docked = True
        self.dock_command_received = False
        self._publish_status(status_label)

    def _return_to_dock(self) -> None:
        """Follow the return path then dock. Called when 'dock' command received."""
        self._publish_status('returning_to_dock_low_battery')
        # Clear the flag so we can still navigate the return path normally
        self.dock_command_received = False
        completed = self._navigate_waypoints(
            self._build_return_waypoints(), 'return'
        )
        if not completed:
            # battery_monitor sent another dock command during the return trip —
            # navigate to pre-dock pose before docking
            self.get_logger().warn(
                'Dock command received during return trip — '
                'navigating to pre-dock pose then docking.'
            )
            self._navigate_to_pre_dock_and_dock('docked')
            return

        self.get_logger().info('Docking…')
        self.navigator.dock()
        self.is_docked = True
        self.dock_command_received = False
        self._publish_status('docked')

    def _wait_for_resume(self) -> None:
        """
        Block until battery_monitor publishes 'resume' (battery >= 95%).
        """
        self.resume_command_received = False
        self._publish_status('charging_waiting_for_resume')
        self.get_logger().info(
            'Waiting for resume command from battery_monitor (battery >= 95%)…'
        )
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=1.0)
            if self.resume_command_received:
                self.resume_command_received = False
                self.get_logger().info('Resume command received — starting patrol.')
                return

    # ── Main run loop ─────────────────────────────────────────────────────────

    def run(self) -> None:
        # ── Step 1: ensure docked, set initial pose ───────────────────────
        if not self.navigator.getDockedStatus():
            self.navigator.info('Not docked on startup — docking first.')
            self.navigator.dock()

        self._initialise_pose()
        self.navigator.waitUntilNav2Active()

        # ── Step 2: wait for battery_monitor's first 'resume' ────────────
        self.get_logger().info(
            'Startup: waiting for battery_monitor to confirm battery >= 95%…'
        )
        self._wait_for_resume()

        loop_count = 0

        while rclpy.ok():
            loop_count += 1
            self.dock_command_received = False
            self._publish_status(f'starting_patrol_loop_{loop_count}')
            self.get_logger().info(f'Starting patrol loop {loop_count}…')

            self.navigator.undock()
            self.is_docked = False

            # ── Outbound leg ──────────────────────────────────────────────
            outbound_ok = self._navigate_waypoints(
                self._build_outbound_waypoints(), 'outbound'
            )

            if not outbound_ok:
                self._return_to_dock()
                self._wait_for_resume()
                continue

            # ── Return leg ────────────────────────────────────────────────
            self._publish_status(f'returning_loop_{loop_count}')
            return_ok = self._navigate_waypoints(
                self._build_return_waypoints(), 'return'
            )

            if not return_ok:
                # Fix for Issue 7: navigate to pre-dock pose before docking
                self.get_logger().warn(
                    'Dock command during return leg — '
                    'navigating to pre-dock pose then docking.'
                )
                self._navigate_to_pre_dock_and_dock('docked_emergency')
                self._wait_for_resume()
                continue

            # ── Full loop done ────────────────────────────────────────────
            self.get_logger().info(f'Patrol loop {loop_count} complete. Docking…')
            self.navigator.dock()
            self.is_docked = True
            self._publish_status(f'completed_loop_{loop_count}')
            self._wait_for_resume()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = PatrolWithBatteryNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Patrol interrupted by user.')
        node.navigator.dock()
        node._publish_status('interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
