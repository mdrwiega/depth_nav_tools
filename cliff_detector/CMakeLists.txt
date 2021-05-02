cmake_minimum_required(VERSION 3.5)
project(cliff_detector)

set(CMAKE_CXX_STANDARD 17)

add_compile_options(-Wall -O3 -Werror)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(include)

add_library(CliffDetectorNode
  src/cliff_detector.cpp
  src/cliff_detector_node.cpp
)
ament_target_dependencies(CliffDetectorNode
  rclcpp
  image_geometry
  image_transport
  sensor_msgs
  geometry_msgs
)

add_executable(cliff_detector
  src/main.cpp
)
target_link_libraries(cliff_detector CliffDetectorNode)

install(TARGETS cliff_detector
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Unit tests
if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(CliffDetectorTest test/cliff_detector_test.cpp)
  target_link_libraries(CliffDetectorTest CliffDetectorNode)
endif()

# Export package
ament_export_dependencies(image_geometry image_transport sensor_msgs geometry_msgs)
ament_export_include_directories(include)
ament_export_libraries(CliffDetectorNode)
ament_package()