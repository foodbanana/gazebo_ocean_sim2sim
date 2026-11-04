cmake_minimum_required(VERSION 3.8)
project(ocean_sim2sim_teleop)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# PICO VR arm teleoperation (port of the Isaac Sim teleop node) + episode recorder. XRoboToolkit binding: its own venv,
# see docs/teleop.md
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/xr_teleop_node.py
  scripts/check_xr_teleop.py
  scripts/record_toggle.py
  scripts/plot_bag_imu.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
