cmake_minimum_required(VERSION 3.5)
project(nav_layer_from_points)

add_compile_options(-Wall -Wextra -Wpedantic)

### Dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

ament_auto_find_build_dependencies()

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
ament_export_dependencies(${PROJECT_NAME} PUBLIC tf2_ros geometry_msgs nav2_costmap_2d)

### Install
ament_auto_package(INSTALL_TO_SHARE
  config
)