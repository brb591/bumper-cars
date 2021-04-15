# Create a Gazebo world with multiple cars
## In one window:
```
source /opt/ros/foxy/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<Path to this repo>/models
cd models
gazebo --verbose -s libgazebo_ros_factory.so
```
## In another window
```
source /opt/ros/foxy/setup.bash
cd python
python3 spawn_red_vehicle.py "redcar1" "/red1" 0 0 0
python3 spawn_red_vehicle.py "redcar2" "/red2" 0 5 0
python3 spawn_red_vehicle.py "redcar3" "/red3" 0 -5 0
python3 spawn_light_blue.py "lb1" "/lb1" 0 -5 0
python3 spawn_light_blue.py "lb2" "/lb2" 3 5 0
python3 spawn_light_blue.py "lb3" "/lb3" -3 0 0
```
## Sending commands to specific cars
### Make the first car go forward
```
source /opt/ros/foxy/setup.bash
ros2 topic pub /red1/cmd_redcar geometry_msgs/Twist '{linear: {x: 1.0}}' -1
```
### Make the second car spin
```
source /opt/ros/foxy/setup.bash
ros2 topic pub /red2/cmd_redcar geometry_msgs/Twist '{angular: {z: 0.25}}' -1
```

### Turn and move at the same time
```
source /opt/ros/foxy/setup.bash
ros2 topic pub /lb2/cmd_lblue geometry_msgs/Twist '{linear: {x: 1.0}, angular: {z: 0.25}}' -1
```

ros2 topic pub /lb1/cmd_lblue geometry_msgs/Twist '{angular: {z: 0.5}}' -1
ros2 topic pub /lb2/cmd_lblue geometry_msgs/Twist '{angular: {z: -1.0}}' -1
ros2 topic pub /lb3/cmd_lblue geometry_msgs/Twist '{angular: {z: -0.25}}' -1

ros2 topic pub /lb1/cmd_lblue geometry_msgs/Twist '{linear: {x: 0.1}}' -1

ros2 topic list -t

## View the onboard camera from a specific cart
The first argument is the vehicle name, the second is the camera topic
```
python3 consume_camera.py lb1 camera1/image_raw
```

# Working on a launch file
source /opt/ros/<ros2-distro>/setup.bash
export TURTLEBOT3_MODEL=waffle

## This looks to be important and will help with specifying meshes?
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/<ros2-distro>/share/turtlebot3_gazebo/models

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    
    
    world = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'worlds', world_file_name)
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])

export BUMPER_CAR_MODEL_PATH=/mnt/c/Users/bbaue/Downloads/dev/bumper-cars/models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$BUMPER_CAR_MODEL_PATH
cd models
gazebo --verbose -s libgazebo_ros_factory.so