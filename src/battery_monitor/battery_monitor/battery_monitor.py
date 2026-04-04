"""Monitor TurtleBot battery and return to dock when charge 
    drops 10% from departure level.
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import BatteryState
from std_msgs.msg import String


class BatteryMonitorNode(Node):
    def __init__(self):
        super().__init__('battery_monitor_node')

        # Fixed amount of battery drop allowed after leaving dock
        self.return_drop = 0.10

        # Tracks whether robot is currently considered docked
        self.is_docked = True

        # Battery percentage recorded when robot leaves dock
        self.departure_percentage = None

        # Dynamic threshold: return to dock when battery falls below this
        self.return_threshold = None

        # Prevent duplicate command publishing
        self.current_mode = None

        self.subscription = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10
        )

        self.command_publisher = self.create_publisher(
            String,
            '/battery_command',
            10
        )

        self.get_logger().info('Battery monitor node started.')

    def battery_callback(self, msg: BatteryState):
        percentage = msg.percentage

        if percentage is None or percentage < 0.0:
            self.get_logger().warn('Received invalid battery percentage.')
            return

        self.get_logger().info(f'Battery percentage: {percentage * 100:.1f}%')

        # If docked and battery is sufficiently charged, tell robot to leave dock
        # and record the battery level at departure.
        if self.is_docked and percentage >= 0.80:
            self.departure_percentage = percentage
            self.return_threshold = max(0.0, self.departure_percentage - self.return_drop)

            self.get_logger().info(
                f'Leaving dock at {self.departure_percentage * 100:.1f}%. '
                f'Return threshold set to {self.return_threshold * 100:.1f}%.'
            )

            if self.current_mode != 'resume':
                self.publish_command('resume')
                self.current_mode = 'resume'

            self.is_docked = False
            return

        # If robot is out working, send it back once it has dropped 10% from departure.
        if (not self.is_docked) and self.return_threshold is not None:
            if percentage <= self.return_threshold:
                self.get_logger().info(
                    f'Battery dropped to {percentage * 100:.1f}%, '
                    f'which is below return threshold of {self.return_threshold * 100:.1f}%.'
                )

                if self.current_mode != 'dock':
                    self.publish_command('dock')
                    self.current_mode = 'dock'

                self.is_docked = True
                self.departure_percentage = None
                self.return_threshold = None

    def publish_command(self, command: str):
        msg = String()
        msg.data = command
        self.command_publisher.publish(msg)
        self.get_logger().info(f'Published command: {command}')


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
