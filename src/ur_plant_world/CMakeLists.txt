cmake_minimum_required(VERSION 3.5)

project(ur_plant_world)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-common5 REQUIRED COMPONENTS profiler)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

# Harmonic
find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
message(STATUS "Compiling against Gazebo Harmonic")

# System plugins (Stage 2).

add_library(ur_plant_world_actuator_dynamics SHARED
  src/actuator_dynamics.cpp)
target_include_directories(ur_plant_world_actuator_dynamics
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(ur_plant_world_actuator_dynamics
  PRIVATE
    gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER})

# The fault_injection plugin target is added in a later Stage 2 step.

install(TARGETS ur_plant_world_actuator_dynamics
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME})

install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
