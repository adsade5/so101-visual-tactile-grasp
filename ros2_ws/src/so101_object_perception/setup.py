from setuptools import find_packages, setup


PACKAGE_NAME = "so101_object_perception"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
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
    zip_safe=True,
    maintainer="so101_visual_tactile_grasp",
    maintainer_email="maintainer@example.com",
    description=(
        "Known-height ArUco workspace object perception "
        "for SO-101."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            (
                "object_pose_node = "
                "so101_object_perception."
                "object_pose_node:main"
            ),
        ],
    },
)