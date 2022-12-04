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
    # urg_launch_path = os.path.join(
    #     get_package_share_directory('urg_node2'),
    #     'launch',
    #     'urg_node2.launch.py'
    # )

    # 起動パラメータファイルのロード
    with open(config_file_path, 'r') as file:
        launch_params = yaml.safe_load(file)['launch']['ros__parameters']

    # URG起動の作成
    # urg_launch = launch.actions.IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([urg_launch_path])
    # )

    # 起動エンティティクラスの作成
    launch_discription = LaunchDescription()

    # 実機での運用
    if(launch_params['real'] is True):
        subprocess.run(['sudo', 'sh', 'src/slcan_add.sh'])
        launch_discription.add_entity(urg_launch)

    # メイン実行機
    main_exec_node = Node(
        package = 'main_executor',
        executable = 'main_executor',
        parameters = [config_file_path],
        output='screen'
    )

    # 起動の追加
    launch_discription.add_entity(main_exec_node)

    return launch_discription
