from setuptools import find_packages, setup


PACKAGE_NAME = "so101_command_gate"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{PACKAGE_NAME}"],
        ),
        (
            f"share/{PACKAGE_NAME}",
            ["package.xml"],
        ),
        (
            f"share/{PACKAGE_NAME}/launch",
            [
                "launch/perception_to_shadow_execution_dry_run.launch.py",
                "launch/perception_to_real_joint_state_gate_dry_run.launch.py",
                "launch/perception_to_connected_shadow_dry_run.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="so101_visual_tactile_grasp",
    maintainer_email="maintainer@example.com",
    description=(
        "Shadow-only final command gate and executor for SO-101 dry-run "
        "grasp trajectories."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "command_gate_node = so101_command_gate.command_gate_node:main",
            "shadow_executor_node = so101_command_gate.shadow_executor_node:main",
            (
                "mock_joint_state_publisher = "
                "so101_command_gate.mock_joint_state_publisher:main"
            ),
            (
                "real_joint_state_bridge_node = "
                "so101_command_gate.real_joint_state_bridge_node:main"
            ),
            (
                "connection_trajectory_node = "
                "so101_command_gate.connection_trajectory_node:main"
            ),
        ],
    },
)
