cmake_minimum_required(VERSION 3.8)
project(ur5_rh_p12_description)

find_package(ament_cmake REQUIRED)

# Let Gazebo resolve package://ur5_rh_p12_description/... mesh URIs
ament_environment_hooks("hooks/resource_paths.dsv.in")
ament_environment_hooks("hooks/resource_paths.sh")

install(DIRECTORY
  config
  meshes
  urdf
  DESTINATION share/${PROJECT_NAME})

ament_package()
