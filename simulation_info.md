# Create a Gazebo world with multiple cars

## Start the simulation: create the arena and populate the bumper cars
The current launch file creates 4 bumper cars, one of each color.  The bumper cars are placed on two different teams.
```
cd python
export BUMPER_CAR_MODEL_PATH=/mnt/c/Users/bbaue/Downloads/dev/bumper-cars/models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$BUMPER_CAR_MODEL_PATH
ros2 launch arena.launch.py
```
## Start the game
The cars are set to inactive by default.  They do nothing until they are made active.  This program sends a ```begin_game``` message to the ```arena/notifications``` topic, which all cars receive at the same time.  This makes them active and they will then start processing the camera feed.
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
## Stop the game
The cars are set back to inactive state.  They will stop moving and stop processing camera frames
```
python3 stop_game_feed.py
```
## Make the bumper car controllers stop running
This will cause the python programs running the controller software to stop running.  It is important to do this before quitting the simulation.  For some reason, quitting the simulation does not cause the controllers to shutdown automatically.
```
python3 shutdown_controllers.py
```
## Quit the simulation
Quitting Gazebo will make the simulation end.  For some reason, hitting ```CTRL-C``` in the terminal window that you used to start the simulation will not kill the ```gzserver``` process, even though it should.

## Make a bumper car move
```
ros2 topic pub /b_team_cart1/cmd_car geometry_msgs/Twist '{linear: {x: -0.3}, angular: {z: -0.15}}' -1
```