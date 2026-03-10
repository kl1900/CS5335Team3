import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Navigator,
    TurtleBot4Directions,
)


class SimPatrolNode(Node):
    def __init__(self):
        super().__init__('sim_patrol_node')

        self.status_pub = self.create_publisher(String, '/patrol_status', 10)
        self.navigator = TurtleBot4Navigator()

    def publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(f'/patrol_status: {text}')

    def run(self) -> None:
        self.publish_status('starting_simulation_patrol')

        # Start docked if possible
        if not self.navigator.getDockedStatus():
            self.get_logger().info('Docking before initializing pose...')
            self.publish_status('docking_before_init')
            self.navigator.dock()

        # Initial pose for the default simulator world
        # These coordinates are for the standard TurtleBot 4 simulation examples.
        initial_pose = self.navigator.getPoseStamped(
            [0.0, 0.0],
            TurtleBot4Directions.NORTH
        )
        self.navigator.setInitialPose(initial_pose)

        self.get_logger().info('Waiting for Nav2 to become active...')
        self.publish_status('waiting_for_nav2')
        self.navigator.waitUntilNav2Active()

        # Waypoints for simulation patrol
        # These are based on the TurtleBot 4 simulator example style.
        patrol_poses = [
            self.navigator.getPoseStamped([0.0, -1.0], TurtleBot4Directions.NORTH),
            self.navigator.getPoseStamped([1.7, -1.0], TurtleBot4Directions.EAST),
            self.navigator.getPoseStamped([1.6, -3.5], TurtleBot4Directions.NORTH),
            self.navigator.getPoseStamped([6.75, -3.46], TurtleBot4Directions.NORTH_WEST),
            self.navigator.getPoseStamped([7.4, -1.0], TurtleBot4Directions.SOUTH),
            self.navigator.getPoseStamped([-1.0, -1.0], TurtleBot4Directions.WEST),
        ]

        self.get_logger().info('Undocking...')
        self.publish_status('undocking')
        self.navigator.undock()

        loop_count = 0

        while rclpy.ok():
            loop_count += 1
            self.get_logger().info(f'Starting patrol loop {loop_count}...')
            self.publish_status(f'patrolling_loop_{loop_count}')

            # Navigate through all patrol points
            self.navigator.startThroughPoses(patrol_poses)

            self.get_logger().info(f'Completed patrol loop {loop_count}.')
            self.publish_status(f'completed_loop_{loop_count}')

        self.publish_status('simulation_patrol_stopped')


def main(args=None):
    rclpy.init(args=args)

    node = SimPatrolNode()

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Simulation patrol interrupted by user.')
        node.publish_status('interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()