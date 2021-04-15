import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'),
        ExecuteProcess(
            cmd=['python3', 'create_arena.py'],
            output='screen'),
        ExecuteProcess(
            cmd=['python3', 'spawn_bumper_car.py', 'lightblue_red', 'a_team_cart1', '/a_team_cart1', '0', '-5', '0'],
            output='screen'),
        ExecuteProcess(
            cmd=['python3', 'spawn_bumper_car.py', 'lime_darkblue', 'a_team_cart2', '/a_team_cart2', '0', '5', '0'],
            output='screen'),
        ExecuteProcess(
            cmd=['python3', 'spawn_bumper_car.py', 'lime_yellow', 'b_team_cart1', '/b_team_cart1', '5', '0', '0'],
            output='screen'),
        ExecuteProcess(
            cmd=['python3', 'spawn_bumper_car.py', 'red_darkblue', 'b_team_cart2', '/b_team_cart2', '-5', '0', '0'],
            output='screen'),
    ])