cmake_minimum_required(VERSION 2.8.3)
project(cliff_detector)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

add_compile_options(-Wall -O3 -Werror)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(depth_nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

add_library(CliffDetectorNode
  src/depth_sensor_pose.cpp
  src/depth_sensor_pose_node.cpp
)
ament_target_dependencies(CliffDetectorNode
  rclcpp
  sensor_msgs
  image_geometry
  image_transport
)

add_executable(cliff_detector
  src/main.cpp
)
target_link_libraries(cliff_detector CliffDetectorNode)

install(TARGETS
  depth_sensor_pose
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

  ament_add_gtest(CliffDetectorTest test/cliff_detector_test.cpp)
  target_link_libraries(CliffDetectorTest CliffDetectorNode)
endif()

# Export package
ament_export_dependencies(image_geometry image_transport sensor_msgs)
ament_export_include_directories(include)
ament_export_libraries(CliffDetectorNode)

ament_package()