cmake_minimum_required(VERSION 3.8)
project(rm_engineer_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find_package(hardware_interface REQUIRED)
# find_package(pluginlib REQUIRED)
# find_package(rclcpp REQUIRED)

# add_library(engineer_hardware SHARED
#   src/engineer_hardware.cpp
# )
# target_include_directories(engineer_hardware PRIVATE include)
# ament_target_dependencies(engineer_hardware
#   hardware_interface
#   pluginlib
#   rclcpp
# )

# pluginlib_export_plugin_description_file(hardware_interface engineer_hardware.xml)

# install(TARGETS
#   engineer_hardware
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# install(FILES engineer_hardware.xml DESTINATION share/${PROJECT_NAME})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(device_interface REQUIRED)

add_executable(pose_subscriber_rpy src/pose_subscriber_rpy.cpp)
ament_target_dependencies(pose_subscriber_rpy
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

add_executable(joint_trajectory_converter src/joint_trajectory_converter.cpp)
ament_target_dependencies(joint_trajectory_converter
  rclcpp
  trajectory_msgs
  device_interface
)

install(TARGETS
  pose_subscriber_rpy
  joint_trajectory_converter
  DESTINATION lib/${PROJECT_NAME}
)


# Launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()