cmake_minimum_required(VERSION 3.8)
project(ocean_sim2sim_drone)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-common5 REQUIRED)
find_package(gz-math7 REQUIRED)

# Gazebo system: drone <-> PX4 bridge lockstep + rotor wrench (src/DroneLockstep.cc)
add_library(OceanSim2simDroneLockstep SHARED src/DroneLockstep.cc)
target_compile_features(OceanSim2simDroneLockstep PRIVATE cxx_std_17)
target_link_libraries(OceanSim2simDroneLockstep PUBLIC
  gz-sim8::core gz-plugin2::register gz-common5::gz-common5 gz-math7::gz-math7)
install(TARGETS OceanSim2simDroneLockstep DESTINATION lib)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/drone_link_test.py
  scripts/px4_bridge_node.py
  scripts/drone_cmd.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES ${PROJECT_NAME}/LICENSE.PegasusSimulator DESTINATION share/${PROJECT_NAME})

# plugin found by Gazebo through GZ_SIM_SYSTEM_PLUGIN_PATH
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")
ament_package()
