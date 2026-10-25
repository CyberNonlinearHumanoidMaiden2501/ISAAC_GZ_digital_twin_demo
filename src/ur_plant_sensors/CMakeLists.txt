cmake_minimum_required(VERSION 3.5)

project(ur_plant_sensors)

find_package(ament_cmake REQUIRED)

# The plant_sensors node and SetFault.srv are added in Stage 2.

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
