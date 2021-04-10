# Create a Gazebo world with multiple cars
## In one window:
```
source /opt/ros/foxy/setup.bash
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