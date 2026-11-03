cmake_minimum_required(VERSION 3.8)
project(ocean_sim2sim_gz_plugins)

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-common5 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)

# Gazebo system: per-link contact forces of the robot arm for the protective stop (src/ContactMonitor.cc)
add_library(OceanSim2simContactMonitor SHARED src/ContactMonitor.cc)
target_compile_features(OceanSim2simContactMonitor PRIVATE cxx_std_17)
target_link_libraries(OceanSim2simContactMonitor PUBLIC
  gz-sim8::core gz-plugin2::register gz-common5::gz-common5 gz-math7::gz-math7
  gz-msgs10::gz-msgs10 gz-transport13::gz-transport13)
install(TARGETS OceanSim2simContactMonitor DESTINATION lib)

# plugin found by Gazebo through GZ_SIM_SYSTEM_PLUGIN_PATH
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")
ament_package()
