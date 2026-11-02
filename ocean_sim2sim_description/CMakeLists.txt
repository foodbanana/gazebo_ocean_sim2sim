cmake_minimum_required(VERSION 3.8)
project(ocean_sim2sim_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME})

ament_package()
