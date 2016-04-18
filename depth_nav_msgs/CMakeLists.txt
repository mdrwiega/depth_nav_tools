cmake_minimum_required(VERSION 2.8.3)
project(depth_nav_msgs)

set(CMAKE_CXX_FLAGS "-std=c++98 ${CMAKE_CXX_FLAGS}")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs std_msgs)

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
    CATKIN_DEPENDS message_runtime geometry_msgs std_msgs
)
