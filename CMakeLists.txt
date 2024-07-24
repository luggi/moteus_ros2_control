cmake_minimum_required(VERSION 3.8)
project(moteus_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(FetchContent)
FetchContent_Declare(
   moteus
   GIT_REPOSITORY https://github.com/mjbots/moteus.git
   GIT_TAG        ee4f6ac9c6391523876836fcf6f276411b7f3670
)

FetchContent_MakeAvailable(moteus)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ament_cmake_auto
)

set(CONTROLLER_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  controller_interface
  realtime_tools
  trajectory_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
ament_auto_find_build_dependencies()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# include moteus hardware libraries


#add_executable(moteus_hardware_interface src/moteus_hardware_interface.cpp)

# add compile directive for shared library
add_library(moteus_hardware_interface SHARED
  src/moteus_hardware_interface.cpp
  controller/gravcomp_controller.cpp
)

target_link_libraries(moteus_hardware_interface PUBLIC
                      moteus::cpp
                      hardware_interface::hardware_interface
                      pluginlib::pluginlib
                      rclcpp::rclcpp
                      rclcpp_lifecycle::rclcpp_lifecycle
                      controller_interface::controller_interface
                      realtime_tools::realtime_tools
                      )

target_include_directories(moteus_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>)

target_compile_features(moteus_hardware_interface PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17



install(TARGETS moteus_hardware_interface
  DESTINATION lib/${PROJECT_NAME})
  
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(moteus_hardware_interface PRIVATE "moteus_hardware_interface_INTERFACE_BUILDING_DLL")

pluginlib_export_plugin_description_file(hardware_interface moteus_hardware_interface.xml)

pluginlib_export_plugin_description_file(controller_interface moteus_hardware_interface.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/moteus_control
)

install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/moteus_control
)

install(
  DIRECTORY description/urdf
  DESTINATION share/moteus_control
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  controller_interface
  realtime_tools
  trajectory_msgs
)
ament_package()
