cmake_minimum_required(VERSION 2.8.3)
project(cliff_detector)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

add_compile_options(
  -Wall
  -O3
  -Werror
)

set(CATKIN_PACKAGE_DEPENDENCIES
  roscpp
  image_geometry
  image_transport
  nodelet
  dynamic_reconfigure
  sensor_msgs
  depth_nav_msgs
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGE_DEPENDENCIES})

# Dynamic reconfigure config
generate_dynamic_reconfigure_options(cfg/CliffDetector.cfg)

find_package( OpenCV REQUIRED )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES CliffDetector CliffDetectorNode
  CATKIN_DEPENDS ${CATKIN_PACKAGE_DEPENDENCIES}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(CliffDetector src/cliff_detector.cpp)
add_dependencies(CliffDetector ${PROJECT_NAME}_gencfg)
target_link_libraries(CliffDetector ${catkin_LIBRARIES})

add_library(CliffDetectorNode src/cliff_detector_node.cpp)
add_dependencies(CliffDetectorNode ${PROJECT_NAME}_gencfg)
target_link_libraries(CliffDetectorNode CliffDetector ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(cliff_detector src/main.cpp)
target_link_libraries(cliff_detector CliffDetectorNode ${catkin_LIBRARIES})
