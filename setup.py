from glob import glob

from setuptools import setup

package_name = "cerebel_orchestrator"

setup(
    name=package_name,
    version="0.2.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/params", glob("params/*.yaml")),
        ("share/" + package_name + "/missions", glob("missions/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="cerebel",
    maintainer_email="thoppay@cerebel.co",
    description="Task-level orchestrator: Nav2 base motion + GR00T policy switching.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "orchestrator = cerebel_orchestrator.orchestrator_node:main",
            "base_adapter = cerebel_orchestrator.base_adapter_node:main",
            "arm_park = cerebel_orchestrator.arm_park:main",
            "probe_robot = cerebel_orchestrator.probe_robot:main",
            "fake_base = cerebel_orchestrator.fake_base:main",
            "fake_arm = cerebel_orchestrator.fake_arm:main",
            "mission_check = cerebel_orchestrator.mission:main",
        ],
    },
)
