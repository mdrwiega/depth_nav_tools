cmake_minimum_required(VERSION 3.5)
project(laserscan_kinect)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

add_compile_options(-Wall -O3 -Werror)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(include)

add_library(LaserScanKinectNode
  src/laserscan_kinect.cpp
  src/laserscan_kinect_node.cpp
)
ament_target_dependencies(LaserScanKinectNode
  rclcpp
  sensor_msgs
  image_geometry
  image_transport
  cv_bridge
)

add_executable(laserscan_kinect
  src/main.cpp
)
target_link_libraries(laserscan_kinect LaserScanKinectNode)

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
  target_link_libraries(laserscanKinectTest LaserScanKinectNode)
endif()

# Export package
ament_export_dependencies(image_geometry image_transport sensor_msgs cv_bridge)
ament_export_include_directories(include)
ament_export_libraries(laserscan_kinect)

ament_package()