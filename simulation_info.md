# Create a Gazebo world with multiple cars

#### Note: All commands are in the `python` director, and all terminal windows have had `source /opt/ros/foxy/setup.bash` run.

## Start the simulation: create the arena and populate the bumper cars
The current launch file creates 4 bumper cars, one of each color.  The bumper cars are placed on two different teams.
```
export BUMPER_CAR_MODEL_PATH=<The path of the model directory on your computer!>
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$BUMPER_CAR_MODEL_PATH
ros2 launch world.launch.py
```
## Start the game
The cars are set to inactive by default.  They do nothing until they are made active.  This program sends a ```begin_game``` message to the ```arena/notifications``` topic, which all cars receive at the same time.  This makes them active and they will then start processing the camera feed.  The code, as written, will make each car randomly make movements.  This should be replaced by actual code to process the camera feed and react to it.
```
python3 start_game.py
```
## To view the camera feed from any bumper car
This allows you to see what any bumper car can see.  You pass the name of the bumper car and the camera you want to view.  Each bumper car has three cameras, `camera_front_sensor`, `camera_right_sensor`, and `camera_left_sensor`.
```
python3 view_camera_feed.py cart1 camera_front_sensor
python3 view_camera_feed.py cart1 camera_left_sensor
python3 view_camera_feed.py cart1 camera_right_sensor
```
## Stop the game
The cars are set back to inactive state.  They will stop moving and stop processing camera frames
```
python3 stop_game.py
```
## Make the bumper car controllers stop running
This will cause the python programs running the controller software to stop running.  It is important to do this before quitting the simulation.  For some reason, quitting the simulation does not cause the controllers to shutdown automatically.
```
python3 shutdown_controllers.py
```
## Quit the simulation
Quitting Gazebo will make the simulation end.  For some reason, hitting ```CTRL-C``` in the terminal window that you used to start the simulation will not kill the ```gzserver``` process, even though it should.