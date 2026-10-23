cmake_minimum_required(VERSION 3.22)
project(openarm_gravity_comp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)

add_executable(gravity_comp_node
  src/arm_dynamics.cpp
  src/gravity_comp_node.cpp
)
add_executable(gravity_comp_calculate
  src/arm_dynamics.cpp
  src/gravity_comp_calculate.cpp
)
target_compile_features(gravity_comp_node PUBLIC cxx_std_17)
target_compile_features(gravity_comp_calculate PUBLIC cxx_std_17)
target_include_directories(gravity_comp_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(gravity_comp_calculate PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(gravity_comp_node
  kdl_parser
  orocos_kdl_vendor
  rclcpp
  rcl_interfaces
  sensor_msgs
  std_msgs
  urdf
)
ament_target_dependencies(gravity_comp_calculate
  kdl_parser
  orocos_kdl_vendor
  urdf
)

install(TARGETS gravity_comp_node gravity_comp_calculate
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(PROGRAMS scripts/validate_mujoco_gravity.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
