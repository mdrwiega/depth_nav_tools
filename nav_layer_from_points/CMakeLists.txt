cmake_minimum_required(VERSION 3.5)
project(nav_layer_from_points)

set(CMAKE_CXX_STANDARD 17)

add_compile_options(-Wall -O3 -Werror)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

include_directories(include)

add_library(nav_layer_from_points
  src/costmap_layer.cpp
)

ament_target_dependencies(nav_layer_from_points
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  tf2_ros
)

install(TARGETS nav_layer_from_points
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export package
ament_export_dependencies(tf2_ros geometry_msgs nav2_costmap_2d)
ament_export_include_directories(include)
ament_export_libraries(laserscan_kinect)
pluginlib_export_plugin_description_file(nav_layer_from_points costmap_plugins.xml)

ament_package()