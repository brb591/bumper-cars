# Create a Gazebo world with multiple cars

## Start the arena and populate the bumper cars
The current launch file creates 4 bumper cars, one of each color.  The bumper cars are placed on two different teams.
```
cd python
export BUMPER_CAR_MODEL_PATH=/mnt/c/Users/bbaue/Downloads/dev/bumper-cars/models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$BUMPER_CAR_MODEL_PATH
ros2 launch arena.launch.py
```
## Start the game
The cars are set to inactive by default.  They do nothing until they are made active.  This program sends a ```begin_game``` message to the ```arena/notifications``` topic, which all cars receive at the same time.  This makes them active and they will then start processing the camera feed.

As of right now, you have to hit CTRL-C to make this stop after 5-10 seconds.  This will need to be fixed so it runs and then exits on its own.
```
python3 start_game_feed.py
```
## To view the camera feed from any bumper car
This allows you to see what any bumper car can see.  You pass the name of the bumper car and the camera you want to view.  Right now, each car only has a single camera.
```
python3 view_camera_feed.py a_team_cart1 camera1
python3 view_camera_feed.py a_team_cart2 camera1
python3 view_camera_feed.py b_team_cart1 camera1
python3 view_camera_feed.py b_team_cart2 camera1
```
## Make a bumper car move
```
ros2 topic pub /b_team_cart1/cmd_car geometry_msgs/Twist '{linear: {x: -0.3}, angular: {z: -0.15}}' -1
```