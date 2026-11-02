cmake_minimum_required(VERSION 3.8)
project(ocean_sim2sim_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/drive_test.py
  scripts/grasp_test.py
  scripts/grasp_demo.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
