import os
import subprocess
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # パラメータファイルのパス設定
    config_file_path = os.path.join(
        get_package_share_directory('main_executor'),
        'config',
        'main_params.yaml'
    )
    # 起動パラメータファイルのロード
    with open(config_file_path, 'r') as file:
        launch_params = yaml.safe_load(file)['launch']['ros__parameters']

    if(launch_params['real'] is True):
        subprocess.run(['sudo', 'sh', 'src/slcan_add.sh'])

    main_exec_node = Node(
        package = 'main_executor',
        executable = 'main_exec',
        parameters = [config_file_path]
    )

    return LaunchDescription([
        main_exec_node
    ])
