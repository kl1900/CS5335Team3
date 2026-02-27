#!/usr/bin/env python3
# Copyright 2022 Clearpath Robotics, Inc.
# Licensed under the Apache License, Version 2.0
# Modified by Pihu, 2026

import rclpy
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before initialising pose')
        navigator.dock()

    # Set initial pose 
    initial_pose = navigator.getPoseStamped([-2.228, -3.413], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()
    
    # Define waypoints
    waypoint1 = navigator.getPoseStamped([-0.546, -0.0759], TurtleBot4Directions.SOUTH_EAST)
    # Same as goal pose for now
    waypoint2 = navigator.getPoseStamped([-8.23, 0.309], TurtleBot4Directions.SOUTH_WEST)

    # Set goal pose 
    # goal_pose = navigator.getPoseStamped([-8.23, 0.309], TurtleBot4Directions.SOUTH_WEST)

    # Undock
    navigator.undock()

    # Go to goal pose
    # navigator.startToPose(goal_pose)
    
    # Go to waypoint1
    navigator.info('Navigating to waypoint1...')
    navigator.startToPose(waypoint1)
    
    # Go to waypoint2    
    navigator.info('Navigating to waypoint2...')
    navigator.startToPose(waypoint2)
    
    # Go back to waypoint1
    navigator.info('Navigating to waypoint1...')
    navigator.startToPose(waypoint1)
    
    # Go back to initial pose
    navigator.info('Navigating closer to dock...')
    navigator.startToPose(initial_pose)
    
    # Return to dock
    navigator.info('Returning to dock...')
    navigator.dock
    
    navigator.info('Docked! Mission complete...')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
