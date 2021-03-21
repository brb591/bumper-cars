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
cd models
python3 spawn_red_vehicle.py "redcar1" "/red1" 0 0 0
python3 spawn_red_vehicle.py "redcar2" "/red2" 0 5 0
python3 spawn_red_vehicle.py "redcar3" "/red3" 0 -5 0
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