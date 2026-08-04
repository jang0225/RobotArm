cmake_minimum_required(VERSION 3.8)
project(robot_arm_controller)

# Let editors consume the exact compiler flags and include paths used by colcon.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(controller_manager_msgs QUIET)
find_package(lifecycle_msgs QUIET)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(fss_interfaces QUIET)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmGripperCommand.msg"
  "msg/GripperCommandCm.msg"
  "msg/JointCommandDegrees.msg"
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(trajectory_validator src/trajectory_validator.cpp)
target_compile_features(trajectory_validator PUBLIC cxx_std_17)
target_include_directories(trajectory_validator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(trajectory_validator builtin_interfaces trajectory_msgs)

add_executable(degree_trajectory_bridge src/degree_trajectory_bridge.cpp)
target_compile_features(degree_trajectory_bridge PUBLIC cxx_std_17)
target_include_directories(degree_trajectory_bridge PRIVATE include)
ament_target_dependencies(degree_trajectory_bridge rclcpp sensor_msgs trajectory_msgs)
target_link_libraries(degree_trajectory_bridge "${cpp_typesupport_target}")

add_executable(gripper_opening_bridge src/gripper_opening_bridge.cpp)
target_compile_features(gripper_opening_bridge PUBLIC cxx_std_17)
target_include_directories(gripper_opening_bridge PRIVATE include)
ament_target_dependencies(gripper_opening_bridge rclcpp trajectory_msgs)
target_link_libraries(gripper_opening_bridge "${cpp_typesupport_target}")

add_executable(fish_motion_node src/fish_motion_node.cpp)
target_compile_features(fish_motion_node PUBLIC cxx_std_17)
target_include_directories(fish_motion_node PRIVATE include)
ament_target_dependencies(fish_motion_node builtin_interfaces rclcpp trajectory_msgs)

set(robot_arm_controller_targets
  degree_trajectory_bridge
  gripper_opening_bridge
  fish_motion_node
)

if(fss_interfaces_FOUND AND controller_manager_msgs_FOUND AND lifecycle_msgs_FOUND)
  add_executable(fss_arm_supervisor_node src/fss_arm_supervisor_node.cpp)
  target_compile_features(fss_arm_supervisor_node PUBLIC cxx_std_17)
  ament_target_dependencies(
    fss_arm_supervisor_node
    builtin_interfaces
    controller_manager_msgs
    fss_interfaces
    lifecycle_msgs
    rclcpp
    sensor_msgs
    std_msgs
    trajectory_msgs
  )
  target_link_libraries(fss_arm_supervisor_node trajectory_validator)
  list(APPEND robot_arm_controller_targets fss_arm_supervisor_node)
  message(STATUS "fss_interfaces found: building fss_arm_supervisor_node")
else()
  message(STATUS "FSS/controller-manager interfaces not found: skipping optional FSS integration node")
endif()

install(TARGETS trajectory_validator ${robot_arm_controller_targets}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_trajectory_validator test/test_trajectory_validator.cpp)
  target_link_libraries(test_trajectory_validator trajectory_validator)
  ament_target_dependencies(test_trajectory_validator trajectory_msgs)
endif()

ament_package()
