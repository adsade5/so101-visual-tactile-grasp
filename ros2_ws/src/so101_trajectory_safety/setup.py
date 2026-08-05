from setuptools import find_packages, setup


PACKAGE_NAME = "so101_trajectory_safety"


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
            ["launch/perception_to_safe_timed_grasp_dry_run.launch.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="so101_visual_tactile_grasp",
    maintainer_email="maintainer@example.com",
    description=(
        "Minimum-jerk time parameterization and offline safety validation "
        "for SO-101 preview grasp trajectories."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            (
                "timed_trajectory_node = "
                "so101_trajectory_safety.timed_trajectory_node:main"
            ),
        ],
    },
)
