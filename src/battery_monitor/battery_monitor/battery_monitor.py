"""Monitor TurtleBot battery and publish dock/resume commands.

Behaviour:
  - Docked + battery >= 95%  → publish 'resume' (leave dock)
  - Patrolling + battery <= 90% → publish 'dock' (return to dock)

Fix for Issue 6:
  is_docked is no longer flipped to True immediately when 'dock' is published.
  Instead, the node listens to /patrol_status and only sets is_docked = True
  when the patrol node confirms it has actually docked (status == 'docked' or
  'docked_emergency'). This prevents the battery monitor from thinking the
  robot is docked while it is still driving home.
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

# Battery thresholds
BATTERY_RESUME_PATROL = 0.95   # Leave dock when charged to this level
BATTERY_RETURN_DOCK   = 0.90   # Return to dock when battery drops to this level


class BatteryMonitorNode(Node):
    def __init__(self):
        super().__init__('battery_monitor_node')

        # Tracks whether robot is currently considered docked.
        # Only flipped to True when patrol node confirms via /patrol_status.
        self.is_docked = True

        # Prevent duplicate command publishing
        self.current_mode = None

        self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10
        )

        # Listen to patrol node to know when docking is actually confirmed
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
        """
        Listen to /patrol_status from patrol_with_battery.py.
        Only mark the robot as truly docked when the patrol node confirms it.
        'docked'           → normal docking after return path completed
        'docked_emergency' → emergency docking mid-return leg
        """
        status = msg.data
        if status in ('docked', 'docked_emergency'):
            self.get_logger().info(
                f'Patrol node confirmed docking (status: {status}). '
                'Marking robot as docked.'
            )
            self.is_docked = True

    def battery_callback(self, msg: BatteryState):
        percentage = msg.percentage

        if percentage is None or percentage < 0.0:
            self.get_logger().warn('Received invalid battery percentage.')
            return

        self.get_logger().info(f'Battery percentage: {percentage * 100:.1f}%')

        # If docked and battery is sufficiently charged, tell robot to leave dock
        if self.is_docked and percentage >= BATTERY_RESUME_PATROL:
            self.get_logger().info(
                f'Battery at {percentage * 100:.1f}% — ready to patrol.'
            )
            if self.current_mode != 'resume':
                self.publish_command('resume')
                self.current_mode = 'resume'
            self.is_docked = False
            return

        # If patrolling and battery has dropped to return threshold, send back to dock.
        # is_docked stays False until patrol_status_callback confirms actual docking.
        if (not self.is_docked) and percentage <= BATTERY_RETURN_DOCK:
            self.get_logger().info(
                f'Battery dropped to {percentage * 100:.1f}% — returning to dock.'
            )
            if self.current_mode != 'dock':
                self.publish_command('dock')
                self.current_mode = 'dock'
            # Do NOT set is_docked = True here

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
