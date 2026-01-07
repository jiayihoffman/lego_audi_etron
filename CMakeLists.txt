cmake_minimum_required(VERSION 3.16)
project(audi_etron)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(hardware_interface REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Note: Python implementation uses 'bleak' library for BLE communication
# Install with: pip install bleak

# INSTALL
install(
  DIRECTORY description/launch description/ros2_control description/urdf description/rviz
  DESTINATION share/audi_etron
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/audi_etron
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(audi_etron_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_audi_etron_launch test/test_view_robot_launch.py)
  ament_add_pytest_test(run_audi_etron_launch test/test_carlikebot_launch.py)
  ament_add_pytest_test(run_audi_etron_launch_remapped test/test_carlikebot_launch_remapped.py)
endif()

## EXPORTS
ament_export_dependencies(hardware_interface)
ament_package()
