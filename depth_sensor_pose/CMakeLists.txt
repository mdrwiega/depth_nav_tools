cmake_minimum_required(VERSION 3.5)
project(depth_sensor_pose)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

add_compile_options(-Wall -O3 -Werror)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

add_definitions(${Eigen_DEFINITIONS})

include_directories(include)

add_library(DepthSensorPoseNode
  src/depth_sensor_pose.cpp
  src/depth_sensor_pose_node.cpp
)
ament_target_dependencies(DepthSensorPoseNode
  rclcpp
  sensor_msgs
  image_geometry
  image_transport
  cv_bridge
)

add_executable(depth_sensor_pose
  src/main.cpp
)
target_link_libraries(depth_sensor_pose DepthSensorPoseNode)

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

  ament_add_gtest(depthSensorPoseTest test/depth_sensor_pose_test.cpp)
  target_link_libraries(depthSensorPoseTest DepthSensorPose)
endif()

# Export package
ament_export_dependencies(image_geometry image_transport sensor_msgs)
ament_export_include_directories(include)
ament_export_libraries(LaserScanKinect LaserScanKinectNode)

ament_package()