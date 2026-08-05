from setuptools import find_packages, setup


PACKAGE_NAME = "so101_frame_transform"


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
    zip_safe=False,
    maintainer="so101_visual_tactile_grasp",
    maintainer_email="maintainer@example.com",
    description=(
        "Workspace-plane to SO-101 base-frame "
        "pose transformation."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            (
                "workspace_to_base_node = "
                "so101_frame_transform."
                "workspace_to_base_node:main"
            ),
            (
                "test_workspace_pose_publisher = "
                "so101_frame_transform."
                "test_workspace_pose_publisher:main"
            ),
        ],
    },
)