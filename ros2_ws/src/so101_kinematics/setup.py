from setuptools import find_packages, setup


PACKAGE_NAME = "so101_kinematics"


setup(
    name=PACKAGE_NAME,
    version="0.2.0",
    packages=find_packages(
        exclude=("test",),
    ),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{PACKAGE_NAME}"],
        ),
        (
            f"share/{PACKAGE_NAME}",
            ["package.xml"],
        ),
    ],
    install_requires=[
        "setuptools",
    ],
    zip_safe=False,
    maintainer="so101_visual_tactile_grasp",
    maintainer_email="maintainer@example.com",
    description=(
        "ROS2 forward kinematics and "
        "consistency checks for SO-101."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            (
                "fk_node = "
                "so101_kinematics.fk_node:main"
            ),
            (
                "test_joint_state_publisher = "
                "so101_kinematics."
                "test_joint_state_publisher:main"
            ),
            (
                "tf_fk_consistency_checker = "
                "so101_kinematics."
                "tf_fk_consistency_checker:main"
            ),
        ],
    },
)