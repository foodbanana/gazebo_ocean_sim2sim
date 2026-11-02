cmake_minimum_required(VERSION 3.8)
project(ocean_sim2sim_gazebo)

find_package(ament_cmake REQUIRED)

# Let Gazebo find the worlds and models in this package (e.g. "gz sim empty_ocean.sdf", model://simple_drone)
ament_environment_hooks("hooks/resource_paths.dsv.in")
ament_environment_hooks("hooks/resource_paths.sh")

install(DIRECTORY
  config
  launch
  models
  worlds
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/build_simple_drone.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
