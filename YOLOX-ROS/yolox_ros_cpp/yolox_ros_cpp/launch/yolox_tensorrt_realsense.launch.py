import launch
import os

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    launch_args = [
        # DeclareLaunchArgument(
        #     'video_device',
        #     default_value='/dev/video0',
        #     description='input video source'
        # ),
        DeclareLaunchArgument(
            'model_path',
            default_value='./ros2-RC24-R2/YOLOX-ROS/weights/tensorrt/yolox_ano3_test3.trt',
            description='yolox model path.'
        ),
        DeclareLaunchArgument(
            'p6',
            default_value='false',
            description='with p6.'
        ),
        DeclareLaunchArgument(
            'class_labels_path',
            default_value='./ros2-RC24-R2/YOLOX-ROS/yolox_ros_cpp/yolox_ros_cpp/labels/ano3_test2.txt',
            description='if use custom model, set class name labels. '
        ),
        DeclareLaunchArgument(
            'num_classes',
            default_value='3',
            description='num classes.'
        ),
        DeclareLaunchArgument(
            'tensorrt/device',
            default_value='0',
            description='GPU index. Set in string type. ex 0'
        ),
        DeclareLaunchArgument(
            'model_version',
            default_value='0.1.1rc0',
            description='yolox model version.'
        ),
        DeclareLaunchArgument(
            'conf',
            default_value='0.80',
            description='yolox confidence threshold.'
        ),
        DeclareLaunchArgument(
            'nms',
            default_value='0.45',
            description='yolox nms threshold'
        ),
        DeclareLaunchArgument(
            'imshow_isshow',
            default_value='true',
            description=''
        ),
        DeclareLaunchArgument(
            'src_image_topic_name',
            default_value='/camera/camera/color/image_raw', #realsense
            description='topic name for source image'
        ),
        DeclareLaunchArgument(
            'publish_image_topic_name',
            default_value='/yolox/image_raw',
            description='topic name for publishing image with bounding box drawn'
        ),
        DeclareLaunchArgument(
            'publish_boundingbox_topic_name',
            default_value='/yolox/realsense',
            description='topic name for publishing bounding box message.'
        ),
    ]

    realsense_file_path = get_package_share_directory('realsense2_camera')

    realsense_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([realsense_file_path + "/launch/rs_launch.py"])
    )

    container = ComposableNodeContainer(
        name='yolox_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='yolox_ros_cpp',
                plugin='yolox_ros_cpp::YoloXNode',
                name='yolox_ros_cpp',
                parameters=[{
                    'model_path': LaunchConfiguration('model_path'),
                    'p6': LaunchConfiguration('p6'),
                    'class_labels_path': LaunchConfiguration('class_labels_path'),
                    'num_classes': LaunchConfiguration('num_classes'),
                    'model_type': 'tensorrt',
                    'model_version': LaunchConfiguration('model_version'),
                    'tensorrt/device': LaunchConfiguration('tensorrt/device'),
                    'conf': LaunchConfiguration('conf'),
                    'nms': LaunchConfiguration('nms'),
                    'imshow_isshow': LaunchConfiguration('imshow_isshow'),
                    'src_image_topic_name': LaunchConfiguration('src_image_topic_name'),
                    'publish_image_topic_name': LaunchConfiguration('publish_image_topic_name'),
                    'publish_boundingbox_topic_name': LaunchConfiguration('publish_boundingbox_topic_name'),
                }],
            ),
        ],
        output='screen',
    )
    return launch.LaunchDescription(
        launch_args + [container, realsense_launch]  #realsense
    )
