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

    
    world_path = os.path.join(os.environ.get('BUMPER_CAR_MODEL_PATH'), 'bumper_cars_world', 'model.sdf')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'),
        ExecuteProcess(
            cmd=['python3', 'create_arena.py'],
            output='screen'),

        # The first cart
        ExecuteProcess(
            cmd=['python3', 'spawn_bumper_car.py', 'red_darkblue', 'cart1', '/cart1', '-2', '0', '0', '0'],
            output='screen'),
        ExecuteProcess(
            cmd=['python3', 'bumper_car_controller.py', 'cart1', 'team1'],
            output='screen'),
        
        # The second cart
        ExecuteProcess(
            cmd=['python3', 'spawn_bumper_car.py', 'lime_darkblue', 'cart2', '/cart2', '4', '0', '0', '0'],
            output='screen'),
        ExecuteProcess(
            cmd=['python3', 'bumper_car_controller.py', 'cart2', 'team1'],
            output='screen'),
        
        # The third cart
        ExecuteProcess(
            cmd=['python3', 'spawn_bumper_car.py', 'lightblue_red', 'cart3', '/cart3', '0', '0', '0', '1.5707'],
            output='screen'),
        ExecuteProcess(
            cmd=['python3', 'bumper_car_controller.py', 'cart3', 'team2'],
            output='screen'),
        
        # The fourth cart
        ExecuteProcess(
            cmd=['python3', 'spawn_bumper_car.py', 'lime_yellow', 'cart4', '/cart4', '2', '0', '0', '-1.5707'],
            output='screen'),
        ExecuteProcess(
            cmd=['python3', 'bumper_car_controller.py', 'cart4', 'team2'],
            output='screen'),
    ])