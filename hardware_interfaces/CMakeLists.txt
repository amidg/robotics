cmake_minimum_required(VERSION 3.16)
project(hardware_interfaces LANGUAGES CXX)

# fetch dependencies
#include(FetchContent)
#
## Specify the library you want to include from GitHub
#FetchContent_Declare(
#  libcreate
#  GIT_REPOSITORY https://github.com/amidg/libcreate.git
#  GIT_TAG        706ddee
#  # Note (Dmitrii): This is required to make it work with `find_package`
#  # https://cmake.org/cmake/help/latest/module/FetchContent.html
#  #OVERRIDE_FIND_PACKAGE
#)
#
## Make sure FetchContent_MakeAvailable is called after declaring all dependencies
#FetchContent_MakeAvailable(libcreate)
##FetchContent_GetProperties(libcreate)
##if (NOT libcreate_POPULATED)
##  FetchContent_Populate(libcreate)
##  add_subdirectory(${libcreate_SOURCE_DIR} ${libcreate_BINARY_DIR})
##endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  libcreate
)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
  message(STATUS "${Dependency} include directory: ${${Dependency}_INCLUDE_DIRS}")
endforeach()


## COMPILE
add_library(
  hardware_interfaces
  SHARED
  src/roomba_control.cpp
)
target_compile_features(hardware_interfaces PUBLIC cxx_std_17)
target_include_directories(hardware_interfaces
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/hardware_interfaces>
)
#target_link_libraries(hardware_interfaces
#    PRIVATE
#)
ament_target_dependencies(
  hardware_interfaces PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(
  hardware_interface
  system_interfaces/roomba_control.xml
)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/hardware_interfaces
)

install(TARGETS hardware_interfaces
  EXPORT export_hardware_interfaces
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  #find_package(ament_cmake_pytest REQUIRED)
  #ament_add_pytest_test(example_2_urdf_xacro test/test_urdf_xacro.py)
  #ament_add_pytest_test(view_example_2_launch test/test_view_robot_launch.py)
  #ament_add_pytest_test(run_example_2_launch test/test_diffbot_launch.py)
endif()

## EXPORTS
ament_export_targets(export_hardware_interfaces HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
