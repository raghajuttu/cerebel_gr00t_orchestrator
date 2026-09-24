"""A park_arms step must name a real profile, whether or not parking is on.

Regression test for a hardware crash. The mission ended with `park_arms home`;
enable_park was false, so the launch never passed a poses file, so only the
built-in "travel" fallback existed; so `start("home")` returned "no park profile
named 'home'" partway through a mission that had otherwise worked.

Two things were wrong and this covers the one that is testable without ROS: the
profiles must load regardless of enable_park. The other -- rclpy raising
"Logger severity cannot be changed between calls" when one call site logs at two
severities -- is covered by the code now using two call sites.
"""

import textwrap

import pytest

yaml = pytest.importorskip("yaml")


def profiles_in(path):
    raw = yaml.safe_load(open(path, encoding="utf-8")) or {}
    return set((raw.get("profiles") or raw).keys())


def test_the_shipped_file_has_every_profile_the_missions_name(tmp_path):
    shipped = profiles_in("params/park_poses.yaml")
    named = set()
    import glob

    for mission in glob.glob("missions/*.yaml"):
        doc = yaml.safe_load(open(mission, encoding="utf-8")) or {}
        for step in doc.get("steps") or []:
            if isinstance(step, dict) and step.get("step") == "park_arms":
                named.add(step.get("profile", "travel"))
    missing = sorted(named - shipped)
    assert not missing, f"missions name park profiles that do not exist: {missing}"


def test_the_launch_loads_poses_even_when_parking_is_disabled():
    """The bug: the poses file was only passed when enable_park was true."""
    source = open("launch/orchestrator.launch.py", encoding="utf-8").read()
    assert 'if not park_poses and get("enable_park") == "true":' not in source, (
        "park poses must load regardless of enable_park -- otherwise only the "
        "fallback profile exists and a park_arms step fails mid-mission"
    )
    assert 'park_poses = get("park_poses_file") or os.path.join(' in source


def test_end_action_logs_at_two_call_sites():
    """rclpy caches severity per call site; one line cannot do info and error."""
    source = open("cerebel_orchestrator/orchestrator_node.py", encoding="utf-8").read()
    assert "level = self.get_logger().info if ok else self.get_logger().error" not in source
    assert 'self.get_logger().info(f"<-- ok: {reason}")' in source
    assert 'self.get_logger().error(f"<-- FAILED: {reason}")' in source


def test_the_park_check_runs_after_the_parker_is_built():
    """Ordering regression: the check used self.parker before it existed.

    The node died with AttributeError before it could start, which no amount of
    mission validation catches -- it is construction order. Checked on the
    source because reproducing it needs a live ROS node.
    """
    source = open("cerebel_orchestrator/orchestrator_node.py", encoding="utf-8").read()
    lines = source.splitlines()

    def line_of(needle):
        for i, line in enumerate(lines):
            if needle in line:
                return i
        raise AssertionError(f"not found in orchestrator_node.py: {needle!r}")

    built = line_of("self.parker = ArmParker(")
    checked = line_of("self._check_mission_park_profiles()")
    assert checked > built, (
        "the park-profile check reads self.parker, so it must come after the "
        f"parker is constructed (built at line {built + 1}, checked at {checked + 1})"
    )


def test_the_envelope_check_does_not_touch_the_parker():
    """It runs early, before the parker exists, and must stay self-contained."""
    source = open("cerebel_orchestrator/orchestrator_node.py", encoding="utf-8").read()
    start = source.index("def _check_mission_envelopes")
    end = source.index("def ", start + 10)
    assert "self.parker" not in source[start:end]
