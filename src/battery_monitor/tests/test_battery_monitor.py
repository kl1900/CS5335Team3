"""Test battery_monitor package.

Updated to match current implementation:
  - resume threshold: 0.95
  - dock threshold:   0.90
  - dock branch only fires when is_docked == False
  - resume branch only fires when is_docked == True
  - is_docked is NOT flipped to True when dock is published
    (that now happens via patrol_status_callback)
"""

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


# ── Resume command tests ──────────────────────────────────────────────────────

def test_publish_resume_at_95_percent():
    """Should publish 'resume' when docked and battery >= 95%."""
    node = BatteryMonitorNode()
    published = []
    node.publish_command = lambda cmd: published.append(cmd)
    node.is_docked = True
    node.current_mode = None

    node.battery_callback(make_battery_msg(0.95))

    assert published == ['resume']
    assert node.current_mode == 'resume'
    assert node.is_docked is False
    node.destroy_node()


def test_publish_resume_above_95_percent():
    """Should publish 'resume' when docked and battery is well above 95%."""
    node = BatteryMonitorNode()
    published = []
    node.publish_command = lambda cmd: published.append(cmd)
    node.is_docked = True
    node.current_mode = None

    node.battery_callback(make_battery_msg(1.0))

    assert published == ['resume']
    assert node.current_mode == 'resume'
    node.destroy_node()


def test_no_resume_below_95_percent():
    """Should NOT publish 'resume' when battery is below 95%."""
    node = BatteryMonitorNode()
    published = []
    node.publish_command = lambda cmd: published.append(cmd)
    node.is_docked = True
    node.current_mode = None

    node.battery_callback(make_battery_msg(0.94))

    assert published == []
    assert node.current_mode is None
    node.destroy_node()


def test_no_resume_when_not_docked():
    """Should NOT publish 'resume' even at high battery if robot is not docked."""
    node = BatteryMonitorNode()
    published = []
    node.publish_command = lambda cmd: published.append(cmd)
    node.is_docked = False
    node.current_mode = None

    node.battery_callback(make_battery_msg(1.0))

    assert published == []
    node.destroy_node()


# ── Dock command tests ────────────────────────────────────────────────────────

def test_publish_dock_at_90_percent():
    """Should publish 'dock' when patrolling and battery drops to 90%."""
    node = BatteryMonitorNode()
    published = []
    node.publish_command = lambda cmd: published.append(cmd)
    node.is_docked = False
    node.current_mode = None

    node.battery_callback(make_battery_msg(0.90))

    assert published == ['dock']
    assert node.current_mode == 'dock'
    node.destroy_node()


def test_publish_dock_below_90_percent():
    """Should publish 'dock' when patrolling and battery is well below 90%."""
    node = BatteryMonitorNode()
    published = []
    node.publish_command = lambda cmd: published.append(cmd)
    node.is_docked = False
    node.current_mode = None

    node.battery_callback(make_battery_msg(0.75))

    assert published == ['dock']
    assert node.current_mode == 'dock'
    node.destroy_node()


def test_no_dock_when_already_docked():
    """Should NOT publish 'dock' when robot is already docked."""
    node = BatteryMonitorNode()
    published = []
    node.publish_command = lambda cmd: published.append(cmd)
    node.is_docked = True
    node.current_mode = None

    node.battery_callback(make_battery_msg(0.80))

    assert published == []
    node.destroy_node()


def test_is_docked_not_set_on_dock_command():
    """
    is_docked must NOT be set to True when dock command is published.
    It should stay False until patrol_status_callback confirms docking.
    This tests the fix for Issue 6.
    """
    node = BatteryMonitorNode()
    node.publish_command = lambda cmd: None  # suppress actual publish
    node.is_docked = False
    node.current_mode = None

    node.battery_callback(make_battery_msg(0.90))

    assert node.is_docked is False  # must still be False
    node.destroy_node()


def test_is_docked_set_on_patrol_status_docked():
    """is_docked should flip to True when patrol_status is 'docked'."""
    node = BatteryMonitorNode()
    node.is_docked = False

    from std_msgs.msg import String
    msg = String()
    msg.data = 'docked'
    node.patrol_status_callback(msg)

    assert node.is_docked is True
    node.destroy_node()


def test_is_docked_set_on_patrol_status_docked_emergency():
    """is_docked should flip to True when patrol_status is 'docked_emergency'."""
    node = BatteryMonitorNode()
    node.is_docked = False

    from std_msgs.msg import String
    msg = String()
    msg.data = 'docked_emergency'
    node.patrol_status_callback(msg)

    assert node.is_docked is True
    node.destroy_node()


def test_is_docked_not_set_on_other_patrol_status():
    """is_docked should NOT flip on unrelated patrol status messages."""
    node = BatteryMonitorNode()
    node.is_docked = False

    from std_msgs.msg import String
    msg = String()
    msg.data = 'patrolling_loop_1'
    node.patrol_status_callback(msg)

    assert node.is_docked is False
    node.destroy_node()


# ── Duplicate suppression tests ───────────────────────────────────────────────

def test_no_repeat_resume_command():
    """Should not publish 'resume' again if current_mode is already 'resume'."""
    node = BatteryMonitorNode()
    published = []
    node.publish_command = lambda cmd: published.append(cmd)
    node.is_docked = True
    node.current_mode = 'resume'

    node.battery_callback(make_battery_msg(1.0))

    assert published == []
    node.destroy_node()


def test_no_repeat_dock_command():
    """Should not publish 'dock' again if current_mode is already 'dock'."""
    node = BatteryMonitorNode()
    published = []
    node.publish_command = lambda cmd: published.append(cmd)
    node.is_docked = False
    node.current_mode = 'dock'

    node.battery_callback(make_battery_msg(0.85))

    assert published == []
    node.destroy_node()


# ── Invalid input tests ───────────────────────────────────────────────────────

def test_ignore_invalid_negative_percentage():
    """Should ignore battery messages with negative percentage."""
    node = BatteryMonitorNode()
    published = []
    node.publish_command = lambda cmd: published.append(cmd)
    node.current_mode = None

    node.battery_callback(make_battery_msg(-1.0))

    assert published == []
    assert node.current_mode is None
    node.destroy_node()


def test_no_publish_between_thresholds():
    """Should not publish anything when battery is between 90% and 95%."""
    node = BatteryMonitorNode()
    published = []
    node.publish_command = lambda cmd: published.append(cmd)
    node.is_docked = False
    node.current_mode = None

    node.battery_callback(make_battery_msg(0.92))

    assert published == []
    assert node.current_mode is None
    node.destroy_node()
