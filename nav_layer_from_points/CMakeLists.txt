cmake_minimum_required(VERSION 2.8.3)
project(nav_layer_from_points)

set(CMAKE_CXX_FLAGS "-std=c++03 ${CMAKE_CXX_FLAGS}")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
  depth_nav_msgs
  dynamic_reconfigure
  geometry_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

# Dynamic reconfigure config
generate_dynamic_reconfigure_options( cfg/NavLayerFromPoints.cfg )

catkin_package(
  	 INCLUDE_DIRS include
    LIBRARIES costmap_layers
    CATKIN_DEPENDS depth_nav_msgs costmap_2d dynamic_reconfigure geometry_msgs
)

add_library(costmap_layers src/costmap_layer.cpp )

add_dependencies(costmap_layers depth_nav_msgs_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(FILES costmap_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS costmap_layers
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
