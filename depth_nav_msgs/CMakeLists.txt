cmake_minimum_required(VERSION 2.8.3)
project(depth_nav_msgs)

set(CMAKE_CXX_STANDARD 14)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_message_files(
   DIRECTORY msg
   FILES
   Point32List.msg
)

## Generate added messages and services with any dependencies listed here
rosidl_generate_interfaces(
   DEPENDENCIES   geometry_msgs std_msgs
)