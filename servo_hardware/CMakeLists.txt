cmake_minimum_required(VERSION 3.16)
project(servo_hardware LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  servo_hardware
  SHARED
  hardware/servo_system.cpp
)
target_compile_features(servo_hardware PUBLIC cxx_std_17)
target_include_directories(servo_hardware PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/servo_hardware>
)
ament_target_dependencies(
  servo_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(servo_hardware PUBLIC serial)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "SERVO_HARDWARE_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface servo_hardware.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/servo_hardware
)
install(
  DIRECTORY description/gazebo description/launch description/ros2_control description/urdf description/rviz
  DESTINATION share/servo_hardware
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/servo_hardware
)
install(TARGETS servo_hardware
  EXPORT export_servo_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_servo_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
