cmake_minimum_required(VERSION 3.20)
project(denso_robot_core_interfaces)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(bcap_core REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "action/DriveString.action"
  "action/DriveValue.action"
  "action/MoveString.action"
  "action/MoveValue.action"
  "msg/ExJoints.msg"
  "msg/Joints.msg"
  "msg/PoseData.msg"
  "msg/UserIO.msg"
  "srv/ChangeMode.srv"
  "srv/GetMode.srv"
  )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
