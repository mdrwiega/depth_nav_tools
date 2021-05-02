cmake_minimum_required(VERSION 3.5)
project(laserscan_kinect)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

add_compile_options(-Wall -O3 -Werror)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

ament_export_dependencies(image_geometry image_transport sensor_msgs)
ament_export_include_directories(include)
ament_export_libraries(LaserScanKinect LaserScanKinectNode)

ament_package()

include_directories(include)

add_library(LaserScanKinect src/laserscan_kinect.cpp)
target_link_libraries(LaserScanKinect)
ament_target_dependencies(LaserScanKinect
  "rclcpp"
  "rcutils"
  "std_msgs"
  "cv_bridge")

add_library(LaserScanKinectNode src/laserscan_kinect_node.cpp)
target_link_libraries(LaserScanKinectNode LaserScanKinect)
ament_target_dependencies(LaserScanKinectNode
  "rclcpp"
  "rcutils"
  "std_msgs"
  "sensor_msgs"
  "image_geometry"
  "image_transport")

add_executable(laserscan_kinect src/main.cpp)
target_link_libraries(laserscan_kinect LaserScanKinectNode)
ament_target_dependencies(laserscan_kinect
  "rclcpp"
  "rcutils"
  "std_msgs"
)

install(TARGETS
  laserscan_kinect
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Unit tests
if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(laserscanKinectTest test/laserscan_kinect_test.cpp)
  target_link_libraries(laserscanKinectTest LaserScanKinect)
  ament_target_dependencies(laserscanKinectTest
    rclcpp sensors_msgs image_geometry)
endif()