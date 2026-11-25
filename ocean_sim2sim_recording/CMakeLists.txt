cmake_minimum_required(VERSION 3.8)
project(ocean_sim2sim_recording)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Boat motion recordings for the ship motion predictor (docs/recording.md): one-command run (sim + bag + meta.json),
# arm motion (fixed / scripted capture-like), bag -> 100 Hz table, E2 sign check
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/record_run.py
  scripts/arm_motion_node.py
  scripts/bag_to_table.py
  scripts/e2_sign_check.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
