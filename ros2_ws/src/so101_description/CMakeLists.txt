cmake_minimum_required(VERSION 3.8)

project(so101_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    urdf
    assets
    launch
    rviz
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  FILES
    model_preparation_manifest.json
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
