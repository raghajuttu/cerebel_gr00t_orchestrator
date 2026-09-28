"""Every true/false launch argument is validated by launch itself.

On 2026-09-28 `use_nav2:=falseenable_park:=true` -- a missing space -- was
accepted: use_nav2 got the whole string, enable_park stayed at its default, and
nothing said so. `choices=` makes launch refuse the value before anything
starts. Read from the source, so this runs without ROS.
"""

import ast
from pathlib import Path

import pytest

LAUNCH_FILES = sorted((Path(__file__).resolve().parents[1] / "launch").glob("*.launch.py"))


def _declarations(path: Path):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if (
            isinstance(node, ast.Call)
            and getattr(node.func, "id", None) == "DeclareLaunchArgument"
        ):
            name = node.args[0].value if node.args else "?"
            kwargs = {kw.arg: kw.value for kw in node.keywords}
            yield name, kwargs


def test_there_are_launch_files_to_check():
    assert LAUNCH_FILES


@pytest.mark.parametrize("path", LAUNCH_FILES, ids=lambda p: p.name)
def test_boolean_arguments_have_choices(path):
    unchecked = []
    for name, kwargs in _declarations(path):
        default = kwargs.get("default_value")
        if isinstance(default, ast.Constant) and default.value in ("true", "false"):
            if "choices" not in kwargs:
                unchecked.append(name)
    assert not unchecked, f"{path.name}: true/false arguments without choices=: {unchecked}"


@pytest.mark.parametrize("path", LAUNCH_FILES, ids=lambda p: p.name)
def test_kinematics_is_one_of_the_shipped_parameter_files(path):
    params = Path(__file__).resolve().parents[1] / "params"
    for name, kwargs in _declarations(path):
        if name != "kinematics":
            continue
        assert "choices" in kwargs, f"{path.name}: kinematics has no choices="
        default = kwargs["default_value"].value
        assert (params / f"nav2_{default}.yaml").exists()
