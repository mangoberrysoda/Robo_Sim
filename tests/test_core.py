import pytest
from sar_swarm.modules.robots.sar_robots import SARRobot 

def test_sar_robot_initial_battery():
    # basic smoke test: new SARRobot exposes a battery_level of 100
    robot = SARRobot(robot_id=1)
    assert hasattr(robot, "battery_level"), "SARRobot must have battery_level attribute"
    assert isinstance(robot.battery_level, (int, float)), "battery_level should be numeric"
    assert robot.battery_level == 100, f"Initial battery level should be 100, got {robot.battery_level}"

def test_sar_robot_ids_and_independent_states():
    # two robots with different ids should be distinct instances and maintain independent battery state
    r1 = SARRobot(robot_id=1)
    r2 = SARRobot(robot_id=2)
    assert r1 is not r2
    assert getattr(r1, "robot_id", None) == 1
    assert getattr(r2, "robot_id", None) == 2

    # mutate r1 battery and ensure r2 remains unchanged
    try:
        r1.battery_level -= 10
    except Exception:
        pytest.skip("battery_level is not writable on this SARRobot implementation")
    assert r1.battery_level == 90
    assert r2.battery_level == 100

def test_sar_robot_class_importable():
    # ensure the class object is importable from the expected path
    cls = SARRobot
    assert isinstance(cls, type), "SARRobot should be a class type"

# @pytest.mark.parametrize("a,b,expected", [
#     (2, 3, 5),
#     (-1, 1, 0),
#     (0, 0, 0),
#     (1.5, 2.5, 4.0),
# ])
# def test_add_function_if_present(a, b, expected):
#     # only run these checks if sar_swarm provides an add function
#     if not hasattr(sar_swarm, "add"):
#         pytest.skip("sar_swarm.add not available in this build")
#     assert sar_swarm.add(a, b) == expected
