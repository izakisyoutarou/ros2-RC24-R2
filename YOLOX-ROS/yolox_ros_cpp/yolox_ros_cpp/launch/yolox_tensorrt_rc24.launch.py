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
        DeclareLaunchArgument(#学習済みデータの指定
            'model_path',
            default_value='/home/kitrp/R2_ws/src/ros2-RC24-R2/YOLOX-ROS/weights/tensorrt/yolox_honban_0501.trt',
            # default_value='/home/kitrp/R2_ws/src/ros2-RC24-R2/YOLOX-ROS/weights/tensorrt/yolox_honban_realsense_0502.trt',
            description='yolox model path.'
        ),
        DeclareLaunchArgument(
            'p6',
            default_value='false',
            description='with p6.'
        ),
        DeclareLaunchArgument(#ラベル情報の指定
            'class_labels_path',
            # default_value='/home/kitrp/R2_ws/src/ros2-RC24-R2/YOLOX-ROS/yolox_ros_cpp/yolox_ros_cpp/labels/ano_honban_0415.txt',
            default_value='/home/kitrp/R2_ws/src/ros2-RC24-R2/YOLOX-ROS/yolox_ros_cpp/yolox_ros_cpp/labels/ano_honban_0429.txt',
            description='if use custom model, set class name labels. '
        ),
        DeclareLaunchArgument(#ラベル数の指定
            'num_classes',
            default_value='4',
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
        DeclareLaunchArgument(#しきい値
            'conf',
            default_value='0.80',
            description='yolox confidence threshold.'
        ),
        DeclareLaunchArgument(#しきい値
            'conf_d455',
            default_value='0.85',
            # default_value='0.50',
            description='yolox confidence threshold.'
        ),
        DeclareLaunchArgument(
            'nms',
            default_value='0.45',
            description='yolox nms threshold'
        ),
        DeclareLaunchArgument(#プレビューの表示
            'imshow_isshow',
            default_value='true',
            description=''
        ),
        DeclareLaunchArgument(#プレビューの表示
            'imshow_isshow_realsense',
            default_value='true',
            description=''
        ),
        DeclareLaunchArgument(#YOLOX-ROSがサブしてる画像トピックのトピック名
            'src_image_topic_name',
            default_value='/image_raw', #C1camera
            description='topic name for source image'
        ),
        DeclareLaunchArgument(#YOLOX-ROSがパブしてる画像トピックのトピック名
            'publish_image_topic_name',
            default_value='/yolox/image_raw_c1',
            description='topic name for publishing image with bounding box drawn'
        ),
        DeclareLaunchArgument(#YOLOX-ROSが推論データをdetection_interfaceにパブしているトピック名
            'publish_boundingbox_topic_name',
            default_value='/yolox/c1',
            description='topic name for publishing bounding box message.'
        ),
        DeclareLaunchArgument(
            'src_image_topic_name_realsense',
            default_value='/camera/d455/color/image_raw', #realsense
            # default_value='/camera/camera/rgbd', #realsense
            description='topic name for source image'
        ),
        DeclareLaunchArgument(
            'publish_image_topic_name_realsense',
            default_value='/yolox/image_raw_realsense',
            description='topic name for publishing image with bounding box drawn'
        ),
        DeclareLaunchArgument(
            'publish_boundingbox_topic_name_realsense',
            default_value='/yolox/realsense',
            description='topic name for publishing bounding box message.'
        ),
        DeclareLaunchArgument(
            'd455_depth_flag',
            default_value='false',
            description='topic name for publishing bounding box message.'
        ),
        DeclareLaunchArgument(
            'c1camera_depth_flag',
            default_value='false',
            description='topic name for publishing bounding box message.'
        ),
    ]

    #c1カメラからトピックをパブできるようしているパッケージ
    camera_file_path = get_package_share_directory('v4l2_camera')

    #v4l2_cameraのlaunchを立ち上げ
    c1_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([camera_file_path + "/launch/c1_camera.launch.py"])
    )
    
    #公式から提供されているrealsenseのパッケージ
    realsense_file_path = get_package_share_directory('realsense2_camera')

    realsense_d455_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([realsense_file_path + "/launch/rs_launch.py"]),
        launch_arguments={
            'camera_name': 'd455',
            'serial_no': "'151422252742'",
            'enable_rgbd': 'true',
            'enable_sync': 'true',
            'align_depth.enable': 'true',
            'enable_color': 'true',
            'enable_depth': 'true',
            'initial_reset': 'true',
        }.items()
    )
    
    realsense_d435i_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([realsense_file_path + "/launch/rs_launch.py"]),
        launch_arguments={
            'camera_name': 'd435i',
            'serial_no': "'843112074106'",
        }.items()
    )

    container = ComposableNodeContainer(
        name='yolox_container_c1',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='yolox_ros_cpp',
                plugin='yolox_ros_cpp::YoloXNode',
                name='yolox_ros_cpp_c1',
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
                    'depth_flag': LaunchConfiguration('c1camera_depth_flag'),
                }],
            ),
        ],
        output='screen',
    )
    
    container_realsense = ComposableNodeContainer(
        name='yolox_container_realsense',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='yolox_ros_cpp',
                plugin='yolox_ros_cpp::YoloXNode',
                name='yolox_ros_cpp_realsense',
                parameters=[{
                    'model_path': LaunchConfiguration('model_path'),
                    'p6': LaunchConfiguration('p6'),
                    'class_labels_path': LaunchConfiguration('class_labels_path'),
                    'num_classes': LaunchConfiguration('num_classes'),
                    'model_type': 'tensorrt',
                    'model_version': LaunchConfiguration('model_version'),
                    'tensorrt/device': LaunchConfiguration('tensorrt/device'),
                    'conf': LaunchConfiguration('conf_d455'),
                    'nms': LaunchConfiguration('nms'),
                    'imshow_isshow': LaunchConfiguration('imshow_isshow_realsense'),
                    'src_image_topic_name': LaunchConfiguration('src_image_topic_name_realsense'),
                    'publish_image_topic_name': LaunchConfiguration('publish_image_topic_name_realsense'),
                    'publish_boundingbox_topic_name': LaunchConfiguration('publish_boundingbox_topic_name_realsense'),
                    'depth_flag': LaunchConfiguration('d455_depth_flag'),
                }],
            ),
        ],
        output='screen',
    )
    return launch.LaunchDescription(
        launch_args +   [
                        container, c1_launch, 
                        # container_realsense, realsense_d455_launch,
                        # realsense_d435i_launch,
                        ]
    )
