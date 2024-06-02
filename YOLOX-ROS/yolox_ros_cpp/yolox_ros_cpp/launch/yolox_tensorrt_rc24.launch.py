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
        ######################################### 学習済みデータ
        DeclareLaunchArgument(#c1cameraの学習済みデータの指定
            'model_path_c1camera',
            default_value='/home/kitrp/R2_ws/src/ros2-RC24-R2/YOLOX-ROS/weights/tensorrt/yolox_honabn_c1_0513.trt',
            # default_value='/home/kitrp/R2_ws/src/ros2-RC24-R2/YOLOX-ROS/weights/tensorrt/yolox_honban_c1_0528.trt',
            description='yolox model path.'
        ),
        DeclareLaunchArgument(#d455の学習済みデータの指定
            'model_path_realsense_d455',
            # default_value='/home/kitrp/R2_ws/src/ros2-RC24-R2/YOLOX-ROS/weights/tensorrt/yolox_honban_realsense_0518.trt',
            default_value='/home/kitrp/R2_ws/src/ros2-RC24-R2/YOLOX-ROS/weights/tensorrt/yolox_honban_realsense_0502.trt',
            description='yolox model path.'
        ),
        #########################################
        
        DeclareLaunchArgument(
            'p6',
            default_value='false',
            description='with p6.'
        ),
        DeclareLaunchArgument(#ラベル情報の指定
            'class_labels_path',
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
        
        ######################################### しきい値
        DeclareLaunchArgument(#c1cameraのしきい値
            'conf_c1camera',
            default_value='0.83',#デフォルト値 0.85
            description='yolox confidence threshold.'
        ),
        DeclareLaunchArgument(#d455のしきい値
            'conf_d455',
            default_value='0.85',#デフォルト値 0.85
            description='yolox confidence threshold.'
        ),
        #########################################
        
        DeclareLaunchArgument(
            'nms',
            default_value='0.45',
            description='yolox nms threshold'
        ),
        ######################################### プレビューの表示
        DeclareLaunchArgument(#c1cameraの表示
            'imshow_isshow_c1camera',
            default_value='false',
            description=''
        ),
        DeclareLaunchArgument(#d455の表示
            'imshow_isshow_realsense_d455',
            default_value='false',
            description=''
        ),
        #########################################
        
        ######################################### C1cameraのトピック名
        DeclareLaunchArgument(#サブしてる画像トピックのトピック名
            'src_image_topic_name_c1camera',
            default_value='/image_raw', #C1camera
            description='topic name for source image'
        ),
        DeclareLaunchArgument(#パブしてる画像トピックのトピック名
            'publish_image_topic_name_c1camera',
            default_value='/yolox/image_raw_c1',
            description='topic name for publishing image with bounding box drawn'
        ),
        DeclareLaunchArgument(#推論データをdetection_interfaceにパブしているトピック名
            'publish_boundingbox_topic_name_c1camera',
            default_value='/yolox/c1',
            description='topic name for publishing bounding box message.'
        ),
        #########################################
        
        ######################################### d455のトピック名
        DeclareLaunchArgument(
            'src_image_topic_name_realsense_d455',
            default_value='/camera/d455/rgbd', 
            description='topic name for source image'
        ),
        DeclareLaunchArgument(
            'publish_image_topic_name_realsense_d455',
            default_value='/yolox/image_raw_realsense_d455',
            description='topic name for publishing image with bounding box drawn'
        ),
        DeclareLaunchArgument(
            'publish_boundingbox_topic_name_realsense_d455',
            default_value='/yolox/realsense_d455',
            description='topic name for publishing bounding box message.'
        ),
        #########################################
    
        
        ######################################### depthを使うかどうか
        DeclareLaunchArgument(
            'depth_flag_c1camera',#これは動かすためのダミー。c1cameraにdepthはない
            default_value='false',
            description='topic name for publishing bounding box message.'
        ),
        DeclareLaunchArgument(
            'depth_flag_d455',
            default_value='true',
            description='topic name for publishing bounding box message.'
        ),
        #########################################
    ]

    #c1カメラからトピックをパブできるようしているパッケージ
    camera_file_path = get_package_share_directory('v4l2_camera')

    #v4l2_cameraのlaunchを立ち上げ
    c1_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([camera_file_path + "/launch/c1_camera.launch.py"])
    )
    
    #公式から提供されているrealsenseのパッケージ
    realsense_file_path = get_package_share_directory('realsense2_camera')

    realsense_d435i_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([realsense_file_path + "/launch/rs_launch.py"]),
        launch_arguments={
            'camera_name': 'd435i',
            'serial_no': "'843112074106'",
            'enable_rgbd': 'true',
            'enable_sync': 'true',
            'align_depth.enable': 'true',
            'enable_color': 'true',
            'enable_depth': 'true',
            'initial_reset': 'true',
        }.items()
    )
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
            # 'initial_reset': 'true',
        }.items()
    )

    container_c1camera = ComposableNodeContainer(
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
                    'model_path': LaunchConfiguration('model_path_c1camera'),
                    'p6': LaunchConfiguration('p6'),
                    'class_labels_path': LaunchConfiguration('class_labels_path'),
                    'num_classes': LaunchConfiguration('num_classes'),
                    'model_type': 'tensorrt',
                    'model_version': LaunchConfiguration('model_version'),
                    'tensorrt/device': LaunchConfiguration('tensorrt/device'),
                    'conf': LaunchConfiguration('conf_c1camera'),
                    'nms': LaunchConfiguration('nms'),
                    'imshow_isshow': LaunchConfiguration('imshow_isshow_c1camera'),
                    'src_image_topic_name': LaunchConfiguration('src_image_topic_name_c1camera'),
                    'publish_image_topic_name': LaunchConfiguration('publish_image_topic_name_c1camera'),
                    'publish_boundingbox_topic_name': LaunchConfiguration('publish_boundingbox_topic_name_c1camera'),
                    'depth_flag': LaunchConfiguration('depth_flag_c1camera'),
                }],
            ),
        ],
        output='screen',
    )
    
    container_realsense_d455 = ComposableNodeContainer(
        name='yolox_container_realsense_d455',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='yolox_ros_cpp',
                plugin='yolox_ros_cpp::YoloXNode',
                name='yolox_ros_cpp_realsense_d455',
                parameters=[{
                    'model_path': LaunchConfiguration('model_path_realsense_d455'),
                    'p6': LaunchConfiguration('p6'),
                    'class_labels_path': LaunchConfiguration('class_labels_path'),
                    'num_classes': LaunchConfiguration('num_classes'),
                    'model_type': 'tensorrt',
                    'model_version': LaunchConfiguration('model_version'),
                    'tensorrt/device': LaunchConfiguration('tensorrt/device'),
                    'conf': LaunchConfiguration('conf_d455'),
                    'nms': LaunchConfiguration('nms'),
                    'imshow_isshow': LaunchConfiguration('imshow_isshow_realsense_d455'),
                    'src_image_topic_name': LaunchConfiguration('src_image_topic_name_realsense_d455'),
                    'publish_image_topic_name': LaunchConfiguration('publish_image_topic_name_realsense_d455'),
                    'publish_boundingbox_topic_name': LaunchConfiguration('publish_boundingbox_topic_name_realsense_d455'),
                    'depth_flag': LaunchConfiguration('depth_flag_d455'),
                }],
            ),
        ],
        output='screen',
    )
    
    return launch.LaunchDescription(
        launch_args +   [
                        container_c1camera, c1_launch, 
                        container_realsense_d455, realsense_d455_launch,
                        realsense_d435i_launch,
                        ]
    )
