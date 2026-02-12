cmake_minimum_required(VERSION 3.8)
project(lidarbot_serial_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================
# Dependencies
# =========================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# =========================
# Library
# =========================

add_library(lidarbot_serial_base SHARED
  src/lidarbot_serial_hardware.cpp
)

target_include_directories(lidarbot_serial_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Proper ROS2 dependency linking
ament_target_dependencies(lidarbot_serial_base
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  std_msgs
  sensor_msgs
)

# =========================
# Install
# =========================

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS lidarbot_serial_base
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Register the plugin with pluginlib
pluginlib_export_plugin_description_file(hardware_interface lidarbot_serial_base.xml)

# =========================
# Export
# =========================

ament_export_include_directories(include)
ament_export_libraries(lidarbot_serial_base)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  std_msgs
  sensor_msgs
)

ament_package()
