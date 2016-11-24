cmake_minimum_required(VERSION 2.8.3)
project(laserscan_kinect)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -O3 ${CMAKE_CXX_FLAGS}")

add_definitions(-DMULTITHREAD=0) # Set to 1 for multithreading

find_package(catkin REQUIRED COMPONENTS 
             roscpp image_geometry image_transport 
             nodelet dynamic_reconfigure sensor_msgs)

# Dynamic reconfigure config
generate_dynamic_reconfigure_options( cfg/LaserscanKinect.cfg )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES LaserScanKinect LaserScanKinectNode LaserScanKinectNodelet
  CATKIN_DEPENDS roscpp dynamic_reconfigure image_geometry image_transport nodelet sensor_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(LaserScanKinect src/laserscan_kinect.cpp)
add_dependencies(LaserScanKinect ${PROJECT_NAME}_gencfg)
target_link_libraries(LaserScanKinect ${catkin_LIBRARIES})

add_library(LaserScanKinectNode src/laserscan_kinect_node.cpp)
add_dependencies(LaserScanKinectNode ${PROJECT_NAME}_gencfg)
target_link_libraries(LaserScanKinectNode LaserScanKinect ${catkin_LIBRARIES})

add_library(LaserScanKinectNodelet src/laserscan_kinect_nodelet.cpp)
target_link_libraries(LaserScanKinectNodelet LaserScanKinectNode ${catkin_LIBRARIES})

add_executable(laserscan_kinect src/main.cpp)
target_link_libraries(laserscan_kinect LaserScanKinectNode ${catkin_LIBRARIES})

# Unit tests
if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(laserscanKinectTest test/laserscan_kinect_test.cpp)
    target_link_libraries(laserscanKinectTest LaserScanKinect)
endif()

