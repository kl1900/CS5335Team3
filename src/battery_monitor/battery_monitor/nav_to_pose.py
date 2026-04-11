#!/usr/bin/env python3
import rclpy
import time
from geometry_msgs.msg import PoseWithCovarianceStamped
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from nav2_simple_commander.robot_navigator import TaskResult

def main():
    rclpy.init()
    navigator = TurtleBot4Navigator()

    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before initialising pose')
        navigator.dock()

    # Set initial pose
    initial_pose = navigator.getPoseStamped([0.266, 0.175], TurtleBot4Directions.NORTH)
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    navigator.initial_pose = initial_pose
    navigator.initial_pose_received = False

    msg = PoseWithCovarianceStamped()
    msg.pose.pose = initial_pose.pose
    msg.header.frame_id = 'map'
    msg.header.stamp = navigator.get_clock().now().to_msg()
    navigator.initial_pose_pub.publish(msg)
    navigator.info('Published initial pose with fresh timestamp')

    time.sleep(2.0)
    rclpy.spin_once(navigator, timeout_sec=2.0)
    navigator.initial_pose_received = True

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Outbound waypoints
    waypoints = [
        navigator.getPoseStamped([-2.29, -1.47], TurtleBot4Directions.NORTH),
        navigator.getPoseStamped([-2.46, -1.8], TurtleBot4Directions.NORTH),
        navigator.getPoseStamped([-1.94, -3.81], TurtleBot4Directions.NORTH),
        navigator.getPoseStamped([2.78, -4.69], TurtleBot4Directions.NORTH),
        navigator.getPoseStamped([2.58, -1.83], TurtleBot4Directions.NORTH),
    ]

    # Return path — reverse route back to dock
    return_path = [
        navigator.getPoseStamped([2.78, -4.69], TurtleBot4Directions.NORTH),
        navigator.getPoseStamped([-1.94, -3.81], TurtleBot4Directions.NORTH),
        navigator.getPoseStamped([-2.46, -1.8], TurtleBot4Directions.NORTH),
        navigator.getPoseStamped([-2.29, -1.47], TurtleBot4Directions.NORTH),
        navigator.getPoseStamped([-0.0992, 0.0579], TurtleBot4Directions.NORTH),
    ]

    # Undock
    navigator.undock()

    # Navigate outbound waypoints
    for waypoint in waypoints:
        navigator.info('Navigating to next waypoint...')
        navigator.startToPose(waypoint)
        while not navigator.isTaskComplete():
            time.sleep(0.5)
        result = navigator.getResult()
        if result != TaskResult.SUCCEEDED:
            navigator.info('Waypoint failed, continuing to next...')
        else:
            navigator.info('Waypoint succeeded!')

    # Navigate return path
    navigator.info('Returning to dock...')
    for waypoint in return_path:
        navigator.info('Navigating return waypoint...')
        navigator.startToPose(waypoint)
        while not navigator.isTaskComplete():
            time.sleep(0.5)
        result = navigator.getResult()
        if result != TaskResult.SUCCEEDED:
            navigator.info('Return waypoint failed, continuing...')
        else:
            navigator.info('Return waypoint succeeded!')

    # Dock
    navigator.info('Docking...')
    navigator.dock()
    navigator.info('Docked! Mission complete.')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
