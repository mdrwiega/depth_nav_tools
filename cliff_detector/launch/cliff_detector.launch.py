import launch
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  ld = launch.LaunchDescription()

  config = os.path.join(
      get_package_share_directory('cliff_detector'),
      'config',
      'params.yaml'
  )

  laserscan_node = Node(
    package='cliff_detector',
    node_executable='cliff_detector',
    parameters=[config],
    remappings=[
      ('/image', '/camera/depth/image_raw'),
      ('/camera_info', '/camera/depth/camera_info'),
    ]
  )
  ld.add_action(laserscan_node)

  return ld