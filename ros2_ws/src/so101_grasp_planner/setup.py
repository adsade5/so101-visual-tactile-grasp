from setuptools import find_packages, setup


PACKAGE_NAME = "so101_grasp_planner"


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
            ["launch/perception_to_grasp_dry_run.launch.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="so101_visual_tactile_grasp",
    maintainer_email="maintainer@example.com",
    description=(
        "Visual target to top-down IK dry-run grasp planner for SO-101."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            (
                "visual_grasp_planner_node = "
                "so101_grasp_planner.visual_grasp_planner_node:main"
            ),
        ],
    },
)
