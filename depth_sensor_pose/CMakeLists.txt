cmake_minimum_required(VERSION 3.5)
project(depth_sensor_pose)

add_compile_options(-Wall -Wextra -Wpedantic)

### Dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

add_definitions(${Eigen_DEFINITIONS})

ament_auto_find_build_dependencies()

### Build
include_directories(include)

ament_auto_add_library(${PROJECT_NAME}
  src/depth_sensor_pose.cpp
  src/depth_sensor_pose_node.cpp
)

ament_auto_add_executable(depth_sensor_pose_exe
  src/main.cpp
)

target_link_libraries(depth_sensor_pose_exe ${PROJECT_NAME})

# Unit tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test
    test/depth_sensor_pose_test.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

# Export package
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_export_dependencies(${PROJECT_NAME} PUBLIC image_geometry image_transport sensor_msgs)

### Install
ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
