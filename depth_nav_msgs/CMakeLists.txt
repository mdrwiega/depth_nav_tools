cmake_minimum_required(VERSION 2.8.3)
project(depth_nav_msgs)

set(CATKIN_PACKAGE_DEPENDENCIES
  message_generation
  std_msgs
  geometry_msgs
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGE_DEPENDENCIES})

add_message_files(
   DIRECTORY msg
   FILES
   Point32List.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES   geometry_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS ${CATKIN_PACKAGE_DEPENDENCIES}
)
