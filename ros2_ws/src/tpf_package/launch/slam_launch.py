import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_explorer = LaunchConfiguration('enable_explorer')
    params_file = LaunchConfiguration('params_file')
    map_output = LaunchConfiguration('map_output')
    rviz_config = LaunchConfiguration('rviz_config')

    pkg_share = get_package_share_directory('tpf_package')
    default_params = os.path.join(pkg_share, 'config', 'slam_defaults.yaml')
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'slam_config.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Usar /clock desde Gazebo o bag'),
        DeclareLaunchArgument(
            'enable_explorer',
            default_value='false',
            description='Levantar o no el nodo de exploracion simple'),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Archivo YAML con parametros para los nodos'),
        DeclareLaunchArgument(
            'map_output',
            default_value='~/ros2_ws/maps/generated_map',
            description='Ruta base (sin extension) para guardar el mapa'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='Archivo RViz para visualizar mapa y TF'),
        Node(
            package='tpf_package',
            executable='slam_node',
            name='slam_node',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]),
        Node(
            package='tpf_package',
            executable='map_saver_node',
            name='map_saver_node',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time, 'map_path': map_output},
            ]),
        Node(
            package='tpf_package',
            executable='explorer_node',
            name='explorer_node',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            condition=IfCondition(enable_explorer)),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}]),
    ])
