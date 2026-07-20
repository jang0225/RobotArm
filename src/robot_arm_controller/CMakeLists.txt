cmake_minimum_required(VERSION 3.8)
project(robot_arm_controller)

# Let editors consume the exact compiler flags and include paths used by colcon.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(dynamixel_sdk REQUIRED)

add_executable(dynamixel_arm_node src/dynamixel_arm_node.cpp)
target_compile_features(dynamixel_arm_node PUBLIC cxx_std_17)
ament_target_dependencies(
  dynamixel_arm_node
  rclcpp
  sensor_msgs
  std_srvs
  dynamixel_sdk
)

install(TARGETS dynamixel_arm_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
