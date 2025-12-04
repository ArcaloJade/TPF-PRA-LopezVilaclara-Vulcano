import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map_yaml')
    enable_rviz = LaunchConfiguration('enable_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    pkg_share = get_package_share_directory('tpf_navigation')
    default_params = os.path.join(pkg_share, 'config', 'navigation_params.yaml')
    default_map = os.path.expanduser(
        '~/TPF-PRA-LopezVilaclara-Vulcano/maps/generated_map.yaml'
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Usar /clock desde Gazebo o bag',
            ),
            DeclareLaunchArgument(
                'params_file',
                default_value=default_params,
                description='Archivo YAML con parametros para los nodos',
            ),
            DeclareLaunchArgument(
                'map_yaml',
                default_value=default_map,
                description='Mapa estatico (.yaml) generado en la Parte A',
            ),
            DeclareLaunchArgument(
                'enable_rviz',
                default_value='false',
                description='Levantar RViz para debug',
            ),
            DeclareLaunchArgument(
                'rviz_config',
                default_value='',
                description='Config RViz (opcional)',
            ),
            Node(
                package='tpf_navigation',
                executable='map_loader_node',
                name='map_loader_node',
                output='screen',
                parameters=[
                    params_file,
                    {'use_sim_time': use_sim_time, 'map_yaml': map_yaml},
                ],
            ),
            Node(
                package='tpf_navigation',
                executable='localization_node',
                name='localization_node',
                output='screen',
                parameters=[params_file, {'use_sim_time': use_sim_time}],
            ),
            Node(
                package='tpf_navigation',
                executable='planner_node',
                name='planner_node',
                output='screen',
                parameters=[params_file, {'use_sim_time': use_sim_time}],
            ),
            Node(
                package='tpf_navigation',
                executable='controller_node',
                name='controller_node',
                output='screen',
                parameters=[params_file, {'use_sim_time': use_sim_time}],
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(enable_rviz),
            ),
        ]
    )
