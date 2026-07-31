cmake_minimum_required(VERSION 3.20)
project(denso_robot_moveit_demo)

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 OLD)
endif()

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
#moveit_package()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_common REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# This is the general demo
add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)

target_include_directories(
  ${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  )

target_link_libraries(
  ${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  moveit_ros_planning_interface::moveit_ros_planning_interface)

# This is the pick-and-place demo with cartesian target poses (6 DOFs robot arms)
add_executable(
  denso_robot_moveit_pickandplace
  src/denso_robot_moveit_pickandplace.cpp)

target_include_directories(
  denso_robot_moveit_pickandplace PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  )

target_link_libraries(
  denso_robot_moveit_pickandplace PUBLIC
  rclcpp::rclcpp
  moveit_ros_planning_interface::moveit_ros_planning_interface)

# This is the pick-and-place demo with joint target positions (4 DOFs robot arms)
add_executable(
  denso_robot_moveit_pickandplace_scara
  src/denso_robot_moveit_pickandplace_scara.cpp)

target_include_directories(
  denso_robot_moveit_pickandplace_scara PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  )

target_link_libraries(
  denso_robot_moveit_pickandplace_scara PUBLIC
  rclcpp::rclcpp
  moveit_ros_planning_interface::moveit_ros_planning_interface)

#=============
# Install
#=============

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include)

install(
  TARGETS denso_robot_moveit_pickandplace
  EXPORT export_denso_robot_moveit_pickandplace
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include)

install(
  TARGETS denso_robot_moveit_pickandplace_scara
  EXPORT export_denso_robot_moveit_pickandplace_scara
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include)

ament_export_targets(
  export_${PROJECT_NAME}
  export_denso_robot_moveit_pickandplace
  export_denso_robot_moveit_pickandplace_scara)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
