from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
  camera = Node(
    package="v4l2_camera",
    executable="v4l2_camera_node",
    parameters=[{#「ls /dev/*」で確認してデバイスを合わせる
      'video_device'     : "/dev/video10",
      "image_size"       : [640, 480],
      "format"           : "rgb"
    }]
  )
  
  image = Node(
    package="rqt_image_view",
    executable="rqt_image_view"
  )

  return LaunchDescription([
    # if you want to disable camera node, remove the following line.
    camera,
    image
  ])
