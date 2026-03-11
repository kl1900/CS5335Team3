"Test battery_monitor package"

import rclpy
from sensor_msgs.msg import BatteryState

from battery_monitor.battery_monitor import BatteryMonitorNode


def setup_module(module):
    rclpy.init()


def teardown_module(module):
    rclpy.shutdown()


def make_battery_msg(percentage: float) -> BatteryState:
    msg = BatteryState()
    msg.percentage = percentage
    return msg


def test_publish_dock_below_20_percent():
    node = BatteryMonitorNode()

    published = []

    def fake_publish_command(command: str):
        published.append(command)

    node.publish_command = fake_publish_command
    node.current_mode = None

    node.battery_callback(make_battery_msg(0.15))

    assert published == ['dock']
    assert node.current_mode == 'dock'

    node.destroy_node()


def test_publish_resume_above_80_percent():
    node = BatteryMonitorNode()

    published = []

    def fake_publish_command(command: str):
        published.append(command)

    node.publish_command = fake_publish_command
    node.current_mode = None

    node.battery_callback(make_battery_msg(0.85))

    assert published == ['resume']
    assert node.current_mode == 'resume'

    node.destroy_node()


def test_no_publish_between_thresholds():
    node = BatteryMonitorNode()

    published = []

    def fake_publish_command(command: str):
        published.append(command)

    node.publish_command = fake_publish_command
    node.current_mode = None

    node.battery_callback(make_battery_msg(0.50))

    assert published == []
    assert node.current_mode is None

    node.destroy_node()


def test_no_repeat_dock_command():
    node = BatteryMonitorNode()

    published = []

    def fake_publish_command(command: str):
        published.append(command)

    node.publish_command = fake_publish_command
    node.current_mode = 'dock'

    node.battery_callback(make_battery_msg(0.10))

    assert published == []

    node.destroy_node()


def test_no_repeat_resume_command():
    node = BatteryMonitorNode()

    published = []

    def fake_publish_command(command: str):
        published.append(command)

    node.publish_command = fake_publish_command
    node.current_mode = 'resume'

    node.battery_callback(make_battery_msg(0.90))

    assert published == []

    node.destroy_node()


def test_ignore_invalid_negative_percentage():
    node = BatteryMonitorNode()

    published = []

    def fake_publish_command(command: str):
        published.append(command)

    node.publish_command = fake_publish_command
    node.current_mode = None

    node.battery_callback(make_battery_msg(-1.0))

    assert published == []
    assert node.current_mode is None

    node.destroy_node()
