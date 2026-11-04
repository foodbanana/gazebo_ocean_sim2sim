cmake_minimum_required(VERSION 3.8)
project(ocean_sim2sim_policy)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# openpi policy client (library: ocean_sim2sim_policy/client.py) + scripts. openpi-client: Python venv, see README
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/policy_client_node.py
  scripts/fake_policy_server.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_client_offline test/test_client_offline.py)
endif()

ament_package()
