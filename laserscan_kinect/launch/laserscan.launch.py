import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  ld = launch.LaunchDescription()
  laserscan_node = Node(
    package='laserscan_kinect',
    node_executable='laserscan_kinect',
    remappings=[
      ('/image', '/camera/depth/image_raw'),
      ('/camera_info', '/camera/depth/camera_info'),
    ]
  )
  ld.add_action(laserscan_node)

  return ld