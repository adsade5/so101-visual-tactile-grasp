from setuptools import setup


PACKAGE_NAME = "so101_mvp_bringup"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=[],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{PACKAGE_NAME}"],
        ),
        (f"share/{PACKAGE_NAME}", ["package.xml", "README.md"]),
        (
            f"share/{PACKAGE_NAME}/launch",
            [
                "launch/mvp_skeleton.launch.py",
                "launch/mvp_hardware_bridge_read_only.launch.py",
                "launch/mvp_hardware_bridge_motion_enabled.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="so101_visual_tactile_grasp",
    maintainer_email="maintainer@example.com",
    description="SO-101 MVP skeleton launch package.",
    license="Apache-2.0",
)
