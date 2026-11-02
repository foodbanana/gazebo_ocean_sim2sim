cmake_minimum_required(VERSION 3.8)
project(ocean_sim2sim_gazebo)

find_package(ament_cmake REQUIRED)

# Let Gazebo find the worlds in this package (e.g. "gz sim empty_ocean.sdf")
ament_environment_hooks("hooks/resource_paths.dsv.in")
ament_environment_hooks("hooks/resource_paths.sh")

install(DIRECTORY
  launch
  worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()
