#!/usr/bin/env python3
# Copyright 2022 Clearpath Robotics, Inc.
# Licensed under the Apache License, Version 2.0
# Modified by Peter Idoko, 2026 - Docking Manager, Team 3
# Purpose: Navigate the TurtleBot4 back to the docking station
#          from any position in the room and dock successfully.

import rclpy
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from nav2_simple_commander.robot_navigator import TaskResult

def main():
    rclpy.init()
    navigator = TurtleBot4Navigator()

    # If already docked, nothing to do
    if navigator.getDockedStatus():
        navigator.info('Robot is already docked. Exiting.')
        rclpy.shutdown()
        return

    # Wait for Nav2 to be fully active before sending any goals
    navigator.waitUntilNav2Active()

    # Define a staging pose near the dock
    # This is a point ~0.5m in front of the dock so the robot
    # approaches from a clean angle before the dock() call takes over.
    # UPDATE these coordinates to match your map's dock location.
    dock_staging_pose = navigator.getPoseStamped(
        [-2.228, -3.413],       # <-- replace with your dock's x, y from your map
        TurtleBot4Directions.NORTH  # <-- replace with the direction the robot faces on the dock
    )

    # Navigate to staging pose from wherever the robot currently is
    navigator.info('Returning to dock staging area...')
    result = navigator.startToPose(dock_staging_pose)

    if result == TaskResult.SUCCEEDED:
        navigator.info('Reached staging area. Initiating dock sequence...')
        navigator.dock()
        
        # Confirm docking success
        if navigator.getDockedStatus():
            navigator.info('Docking successful! Mission complete.')
        else:
            navigator.error('Docking failed — robot may need manual intervention.')
    
    elif result == TaskResult.CANCELED:
        navigator.error('Navigation to dock was canceled.')
    
    elif result == TaskResult.FAILED:
        navigator.error('Failed to reach dock staging area. Check costmap and localization.')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
