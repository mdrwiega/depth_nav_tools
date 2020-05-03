cmake_minimum_required(VERSION 2.8.3)
project(depth_sensor_pose)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

add_compile_options(
  -Wall
  -O3
  # -Werror
)

set(CATKIN_PACKAGE_DEPENDENCIES
  roscpp
  image_geometry
  image_transport
  nodelet
  dynamic_reconfigure
  sensor_msgs
  pcl_ros
  cmake_modules
  costmap_2d
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGE_DEPENDENCIES})

# Dynamic reconfigure config
generate_dynamic_reconfigure_options( cfg/DepthSensorPose.cfg )

find_package( PCL REQUIRED )
find_package( Eigen3 REQUIRED )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES DepthSensorPose DepthSensorPoseNode
  CATKIN_DEPENDS ${CATKIN_PACKAGE_DEPENDENCIES}
)

add_definitions(${Eigen_DEFINITIONS})

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_library(DepthSensorPose src/depth_sensor_pose.cpp)
add_dependencies(DepthSensorPose ${PROJECT_NAME}_gencfg)
target_link_libraries(DepthSensorPose ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Eigen_LIBRARIES})

add_library(DepthSensorPoseNode src/depth_sensor_pose_node.cpp)
add_dependencies(DepthSensorPoseNode ${PROJECT_NAME}_gencfg)
target_link_libraries(DepthSensorPoseNode DepthSensorPose ${catkin_LIBRARIES})


add_executable(depth_sensor_pose src/main.cpp)
target_link_libraries(depth_sensor_pose DepthSensorPoseNode ${catkin_LIBRARIES})
