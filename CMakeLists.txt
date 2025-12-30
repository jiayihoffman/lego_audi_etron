cmake_minimum_required(VERSION 3.16)
project(audi_etron LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  audi_etron
  SHARED
  hardware/carlikebot_system.cpp
)
target_compile_features(audi_etron PUBLIC cxx_std_17)
target_include_directories(audi_etron PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/audi_etron>
)
ament_target_dependencies(
  audi_etron PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface audi_etron.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/audi_etron
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf description/rviz
  DESTINATION share/audi_etron
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/audi_etron
)
install(TARGETS audi_etron
  EXPORT export_audi_etron
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(audi_etron_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_audi_etron_launch test/test_view_robot_launch.py)
  ament_add_pytest_test(run_audi_etron_launch test/test_carlikebot_launch.py)
  ament_add_pytest_test(run_audi_etron_launch_remapped test/test_carlikebot_launch_remapped.py)
endif()

## EXPORTS
ament_export_targets(export_audi_etron HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
