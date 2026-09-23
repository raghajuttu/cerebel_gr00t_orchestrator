"""Task-level orchestration for the bimanual GR00T robot on a mobile base.

The modules split cleanly in two:

*pure* (no ROS, unit-testable anywhere)
    mission.py        the mission file: stations, policies, steps
    state_machine.py  what to do next, retries, repeats, terminal states
    phase_monitor.py  when a policy phase is finished
    envelopes.py      named joint envelopes, and the check_arms measurement

*ROS* (needs rclpy on the robot computer)
    orchestrator_node.py  the driver that ties the three together
    nav_client.py         Nav2 NavigateToPose
    policy_runner.py      starting and stopping a GR00T policy session
    arm_park.py           ramped move of the arms to a parked pose
    base_adapter_node.py  vendor chassis topics -> the interface Nav2 expects
    probe_robot.py        read-only survey of what the robot actually exposes
"""

__version__ = "0.3.0"
