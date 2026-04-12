#!/usr/bin/env python3
import rclpy
import time
from geometry_msgs.msg import PoseWithCovarianceStamped
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

def main():
    rclpy.init()
    navigator = TurtleBot4Navigator()

    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before initialising pose')
        navigator.dock()

    # Set initial pose with current timestamp
    initial_pose = navigator.getPoseStamped([0.266, 0.175], TurtleBot4Directions.NORTH)
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    navigator.initial_pose = initial_pose
    navigator.initial_pose_received = False

    # Publish initial pose manually with fresh timestamp
    msg = PoseWithCovarianceStamped()
    msg.pose.pose = initial_pose.pose
    msg.header.frame_id = 'map'
    msg.header.stamp = navigator.get_clock().now().to_msg()
    navigator.initial_pose_pub.publish(msg)
    navigator.info('Published initial pose with fresh timestamp')

    time.sleep(2.0)
    rclpy.spin_once(navigator, timeout_sec=2.0)

    navigator.initial_pose_received = True  # Force proceed if AMCL already has pose

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    waypoint1 = navigator.getPoseStamped([-2.29, -1.47], TurtleBot4Directions.NORTH)

    navigator.undock()

    navigator.info('Navigating to waypoint1...')
    navigator.startToPose(waypoint1)

    # Navigate back to dock staging position first
    navigator.info('Returning to dock area...')
    # Staging pose ~1 meter in front of dock in clear space
    dock_staging = navigator.getPoseStamped([-0.0992, 0.0579], TurtleBot4Directions.NORTH)
    navigator.startToPose(dock_staging)

    # Then trigger the dock mechanism
    navigator.info('Docking...')
    navigator.dock()

    navigator.info('Docked! Mission complete.')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
