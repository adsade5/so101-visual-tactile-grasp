from setuptools import find_packages, setup


PACKAGE_NAME = "so101_mvp_control"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{PACKAGE_NAME}"],
        ),
        (f"share/{PACKAGE_NAME}", ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="so101_visual_tactile_grasp",
    maintainer_email="maintainer@example.com",
    description="SO-101 MVP control skeleton nodes.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mvp_grasp_controller_node = so101_mvp_control.mvp_grasp_controller_node:main",
            "mvp_hardware_bridge_node = so101_mvp_control.mvp_hardware_bridge_node:main",
        ],
    },
)

