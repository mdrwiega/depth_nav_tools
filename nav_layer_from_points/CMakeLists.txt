cmake_minimum_required(VERSION 3.5)
project(nav_layer_from_points)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### Dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

ament_auto_find_build_dependencies()

set(dep_pkgs
  rclcpp
  tf2_ros
  geometry_msgs
  nav2_costmap_2d
)

### Build
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/costmap_layer.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Export package
ament_target_dependencies(${PROJECT_NAME} ${dep_pkgs})
ament_export_dependencies(${dep_pkgs})
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

### Install
ament_auto_package(INSTALL_TO_SHARE
  config
)