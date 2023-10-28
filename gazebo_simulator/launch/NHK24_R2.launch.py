import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

# this is the function launch  system will look for

def generate_launch_description():
    package_name = 'gazebo_simulator'
    world_file_name = 'NHK23_court.world'
    urdf_file_name = 'rr.urdf'

    config_file_path = os.path.join(
        get_package_share_directory('main_executor'),
        'config',
        'main_params.yaml'
    )

    with open(config_file_path, 'r') as file:
        initial_pose = yaml.safe_load(file)['/**']['ros__parameters']['initial_pose']

    # オイラー角からクオータニオンにする - https://ken3susume.com/archives/12958
    initial_pose_ = 'position: {{x: {:.2f}, y: {:.2f}, z: 0.1}}, orientation: {{x: 0.0, y: 0.0, z: 0.7, w: 0.7}}'.format(initial_pose[0], initial_pose[1], initial_pose[2])

    # full  path to urdf and world file

    world = os.path.join(get_package_share_directory(package_name), 'worlds', world_file_name)

    urdf = os.path.join(get_package_share_directory(package_name), 'urdf', urdf_file_name)

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # read urdf contents because to spawn an entity in
    # gazebo we need to provide entire urdf as string on  command line

    xml = open(urdf, 'r').read()

    # double quotes need to be with escape sequence
    xml = xml.replace('"', '\\"')

    # this is argument format for spwan_entity service
    spwan_args = '{name: \"r2\", xml: \"'  +  xml + '\" , initial_pose: {' + initial_pose_ + '}}'

    # create and return launch description object
    return LaunchDescription([

        # start gazebo, notice we are using libgazebo_ros_factory.so instead of libgazebo_ros_init.so
        # That is because only libgazebo_ros_factory.so contains the service call to /spawn_entity
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        # tell gazebo to spwan your robot in the world by calling service
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', spwan_args],
            output='screen'),
        Node(
            package='gazebo_simulator',
            executable='planar_bot_converter',
            output='screen',
            parameters=[config_file_path]
        ),
    ])
