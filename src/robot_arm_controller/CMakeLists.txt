cmake_minimum_required(VERSION 3.8)
project(robot_arm_controller)

# Let editors consume the exact compiler flags and include paths used by colcon.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointCommandDegrees.msg"
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(degree_trajectory_bridge src/degree_trajectory_bridge.cpp)
target_compile_features(degree_trajectory_bridge PUBLIC cxx_std_17)
target_include_directories(degree_trajectory_bridge PRIVATE include)
ament_target_dependencies(degree_trajectory_bridge rclcpp trajectory_msgs)
target_link_libraries(degree_trajectory_bridge "${cpp_typesupport_target}")

add_executable(fish_motion_node src/fish_motion_node.cpp)
target_compile_features(fish_motion_node PUBLIC cxx_std_17)
target_include_directories(fish_motion_node PRIVATE include)
ament_target_dependencies(fish_motion_node builtin_interfaces rclcpp trajectory_msgs)

install(TARGETS degree_trajectory_bridge fish_motion_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
