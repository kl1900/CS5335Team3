"""Monitor TurtleBot battery using relative percentage changes.

Behaviour:
  - Records battery level at undock (departure level)
  - Sends 'dock' when battery drops 5% from departure level
  - Records battery level at docking (docked level)  
  - Sends 'resume' when battery rises 5% from docked level
  - Listens to /patrol_status to know actual dock/undock state
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

DROP_THRESHOLD = 0.05   # 5% drop triggers dock command
RISE_THRESHOLD = 0.05   # 5% rise triggers resume command


class BatteryMonitorNode(Node):
    def __init__(self):
        super().__init__('battery_monitor_node')

        self.is_docked = True
        self.current_mode = None

        # Battery level recorded at moment of undocking
        self.departure_level = None

        # Battery level recorded at moment of docking
        self.docked_level = None

        self.create_subscription(
            BatteryState,
            '/turtlebot468/battery_state',
            self.battery_callback,
            10
        )

        self.create_subscription(
            String,
            '/patrol_status',
            self.patrol_status_callback,
            10
        )

        self.command_publisher = self.create_publisher(
            String,
            '/battery_command',
            10
        )

        self.get_logger().info('Battery monitor node started.')

    def patrol_status_callback(self, msg: String) -> None:
        status = msg.data

        # Robot has confirmed it is docked
        if status in ('docked', 'docked_emergency'):
            self.get_logger().info(
                f'Patrol node confirmed docking (status: {status}).'
            )
            self.is_docked = True
            self.departure_level = None  # Reset for next patrol

        # Robot has confirmed it has undocked and is patrolling
        elif status.startswith('starting_patrol_loop'):
            self.get_logger().info(
                'Patrol node confirmed undocking — recording departure battery level.'
            )
            self.is_docked = False

    def battery_callback(self, msg: BatteryState):
        percentage = msg.percentage

        if percentage is None or percentage < 0.0:
            self.get_logger().warn('Received invalid battery percentage.')
            return

        self.get_logger().info(f'Battery: {percentage * 100:.1f}%')

        if self.is_docked:
            # Record docked level for resume threshold
            self.docked_level = percentage

            # Send resume when battery has risen 5% from docked level
            if self.docked_level is not None:
                if percentage >= self.docked_level + RISE_THRESHOLD:
                    if self.current_mode != 'resume':
                        self.get_logger().info(
                            f'Battery rose 5% to {percentage * 100:.1f}% — '
                            f'publishing resume.'
                        )
                        self.publish_command('resume')
                        self.current_mode = 'resume'
        else:
            # Record departure level on first reading after undocking
            if self.departure_level is None:
                self.departure_level = percentage
                self.get_logger().info(
                    f'Departure level recorded: {percentage * 100:.1f}%'
                )
                return

            # Send dock when battery drops 5% from departure level
            drop = self.departure_level - percentage
            if drop >= DROP_THRESHOLD:
                if self.current_mode != 'dock':
                    self.get_logger().info(
                        f'Battery dropped 5% to {percentage * 100:.1f}% — '
                        f'publishing dock.'
                    )
                    self.publish_command('dock')
                    self.current_mode = 'dock'

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
