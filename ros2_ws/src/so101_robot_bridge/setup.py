from setuptools import find_packages, setup


PACKAGE_NAME = "so101_robot_bridge"


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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="adsade5",
    maintainer_email="adsade5@users.noreply.github.com",
    description=(
        "Safe localhost TCP bridge from a simulated or real "
        "LeRobot hardware server to ROS 2."
    ),
    license="MIT",
    entry_points={
        "console_scripts": [
            (
                "tcp_bridge_node = "
                "so101_robot_bridge.tcp_bridge_node:main"
            ),
        ],
    },
)