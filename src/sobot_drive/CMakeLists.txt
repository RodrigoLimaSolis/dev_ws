cmake_minimum_required(VERSION 3.16)
project(sobot_drive LANGUAGES CXX)

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
  sobot_drive
  SHARED
  hardware/diffbot_system.cpp
)
target_compile_features(sobot_drive PUBLIC cxx_std_17)
target_include_directories(sobot_drive PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/sobot_drive>
)
ament_target_dependencies(
  sobot_drive PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(sobot_drive PUBLIC serial)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "SOBOT_DRIVE_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface sobot_drive.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/sobot_drive
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf description/meshes
  DESTINATION share/sobot_drive
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/sobot_drive
)
install(TARGETS sobot_drive
  EXPORT export_sobot_drive
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_sobot_drive HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
