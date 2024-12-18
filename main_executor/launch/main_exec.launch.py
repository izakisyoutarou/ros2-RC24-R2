import os
import subprocess
import yaml
import launch
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # パラメータファイルのパス設定
    config_file_path = os.path.join(
        get_package_share_directory('main_executor'),
        'config',
        'main_params.yaml'
    )
    # URG起動ファイルのパス設定
    urg_launch_path = os.path.join(
        get_package_share_directory('urg_node2'),
        'launch',
        'urg_node2_2lidar.launch.py'
    )
    # USB CAN起動ファイルのパス設定
    slcan_launch_path = os.path.join(
        get_package_share_directory('socketcan_interface'),
        'config',
        'slcan_add.sh'
    )

    # 点群統合pkg起動ファイルのパス設定
    merger_launch_path = os.path.join(
        get_package_share_directory('ros2_laser_scan_merger'),
        'launch',
        'merge_2_scan.launch.py'
    )
    # yolox_c1,yolox_realsense起動ファイルのパス設定
    yolox_launch_path = os.path.join(
        get_package_share_directory('yolox_ros_cpp'),
        'launch',
        'yolox_tensorrt_rc24.launch.py'
    )

    # 起動パラメータファイルのロード
    with open(config_file_path, 'r') as file:
        launch_params = yaml.safe_load(file)['launch']['ros__parameters']

    # メイン実行機ノードの作成
    main_exec_node = Node(
        package = 'main_executor',
        executable = 'main_exec',
        parameters = [config_file_path],
        output='screen'
    )
    # URG起動の作成
    urg_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([urg_launch_path])
    )

    # 点群統合pkg起動の作成
    merger_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([merger_launch_path])
    )

    # 軌道計画機ノードの作成
    trajectory_planner_node = Node(
        package = 'spline_pid',
        executable = 'R2_trajectories.py',
        parameters= [config_file_path],
        output='screen'
    )
    
    # yolox_c1,yolox_realsenseノードの作成
    yolox_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([yolox_launch_path])
    )

    # 起動エンティティクラスの作成
    launch_discription = LaunchDescription()

    # 起動の追加
    if(launch_params['slcan'] is True):
        subprocess.run(['sudo', 'sh', slcan_launch_path])
    if(launch_params['scan'] is True):
        launch_discription.add_entity(urg_launch)
    if(launch_params['trajectory_planner'] is True):
        launch_discription.add_entity(trajectory_planner_node)
    if(launch_params['yolox_launch'] is True):
        launch_discription.add_entity(yolox_launch)

    launch_discription.add_entity(merger_launch)
    
    launch_discription.add_entity(main_exec_node)

    return launch_discription