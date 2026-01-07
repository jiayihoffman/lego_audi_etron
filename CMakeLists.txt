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

# Find SimpleBLE for LEGO motor control
# SimpleBLE is not available in standard Linux package repositories and must be built from source.
# macOS: Available via Homebrew (brew install simpleble)
# Linux: Must be built from source - see instructions below

find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(SIMPLEBLE QUIET simpleble)
  # D-Bus is required by SimpleBLE on Linux
  pkg_check_modules(DBUS QUIET dbus-1)
endif()

# Try to find SimpleBLE using find_path/find_library if pkg-config didn't work
if(NOT SIMPLEBLE_FOUND)
  find_path(SIMPLEBLE_INCLUDE_DIR NAMES simpleble/SimpleBLE.h)
  find_library(SIMPLEBLE_LIBRARY NAMES simpleble)
  if(SIMPLEBLE_INCLUDE_DIR AND SIMPLEBLE_LIBRARY)
    set(SIMPLEBLE_FOUND TRUE)
    set(SIMPLEBLE_INCLUDE_DIRS ${SIMPLEBLE_INCLUDE_DIR})
    set(SIMPLEBLE_LIBRARIES ${SIMPLEBLE_LIBRARY})
  endif()
endif()

if(NOT SIMPLEBLE_FOUND)
  message(FATAL_ERROR "")
  message(FATAL_ERROR "================================================================================")
  message(FATAL_ERROR "SimpleBLE not found. LEGO motor control requires SimpleBLE.")
  message(FATAL_ERROR "================================================================================")
  message(FATAL_ERROR "")
  message(FATAL_ERROR "Installation instructions:")
  message(FATAL_ERROR "")
  message(FATAL_ERROR "  macOS:")
  message(FATAL_ERROR "    brew install simpleble")
  message(FATAL_ERROR "")
  message(FATAL_ERROR "  Linux (Ubuntu/Debian):")
  message(FATAL_ERROR "    1. Install dependencies:")
  message(FATAL_ERROR "       sudo apt-get update")
  message(FATAL_ERROR "       sudo apt-get install libdbus-1-dev cmake build-essential")
  message(FATAL_ERROR "")
  message(FATAL_ERROR "    2. Build and install SimpleBLE:")
  message(FATAL_ERROR "       git clone https://github.com/OpenBluetoothToolbox/SimpleBLE.git")
  message(FATAL_ERROR "       cd SimpleBLE/simpleble")
  message(FATAL_ERROR "       mkdir build && cd build")
  message(FATAL_ERROR "       cmake .. -DCMAKE_BUILD_TYPE=Release")
  message(FATAL_ERROR "       cmake --build . --parallel")
  message(FATAL_ERROR "       sudo cmake --install .")
  message(FATAL_ERROR "       sudo ldconfig")
  message(FATAL_ERROR "")
  message(FATAL_ERROR "  Note: SimpleBLE requires CMake >= 3.21")
  message(FATAL_ERROR "")
  message(FATAL_ERROR "For more details, see: https://github.com/OpenBluetoothToolbox/SimpleBLE")
  message(FATAL_ERROR "")
else()
  message(STATUS "SimpleBLE found: ${SIMPLEBLE_LIBRARIES}")
endif()

## COMPILE
add_library(
  audi_etron
  SHARED
  hardware/carlikebot_system.cpp
  hardware/lego_motor_controller.cpp
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

# Link SimpleBLE if found
if(SIMPLEBLE_FOUND)
  target_include_directories(audi_etron PRIVATE ${SIMPLEBLE_INCLUDE_DIRS})
  target_link_libraries(audi_etron PRIVATE ${SIMPLEBLE_LIBRARIES})
  
  # Link D-Bus (required by SimpleBLE on Linux)
  if(DBUS_FOUND)
    target_include_directories(audi_etron PRIVATE ${DBUS_INCLUDE_DIRS})
    target_link_libraries(audi_etron PRIVATE ${DBUS_LIBRARIES})
    target_compile_options(audi_etron PRIVATE ${DBUS_CFLAGS_OTHER})
  endif()
endif()

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

