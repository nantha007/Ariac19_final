# Final project.

# Todo

- Adjusting the flipping without collision.
- Conveyer parts list integration.
- Drop parts handling.

Link to the video -->



## Running the package:

- Unpack the zip file "group3_final_project.zip" into "group3_final_project" folder into the workspace src directory.
- Open two terminals.
- In terminal-1:
```
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash (Hoping the folder is in home directory)
roslaunch group3_final_project group3_final_project.launch
```
- In terminal-2:
```
source /opt/ros/melodic/setup.bash
source ~/ariac_ws/install/setup.bash (The ariac workspace where moveit package is present)
roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm1
```
- In terminal-3:
```
source /opt/ros/melodic/setup.bash
source ~/ariac_ws/install/setup.bash (The ariac workspace where moveit package is present)
roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm2
```
- In terminal-4:
```
source /opt/ros/melodic/setup.bash
cd ~/catkin_ws/
catkin_make --only-pkg-with-deps group3_final_project
source ~/catkin_ws/devel/setup.bash (Hoping the folder is in home directory)
rosrun group3_final_project ariac_example_node
```

## Watch Conveyor Node

To test the watch_conveyor code

-In terminal 1:

	source /opt/ros/melodic/setup.bash
	cd ~/catkin_ws/
	catkin_make --only-pkg-with-deps group3_final_project
	source ~/catkin_ws/devel/setup.bash (Hoping the folder is in home directory)
	roslaunch group3_final_project watch_conveyor.launch 

-In terminal 2:
	
	source /opt/ros/melodic/setup.bash
	source ~/catkin_ws/devel/setup.bash
	rosrun group3_final_project watch_conveyor_node

	




	
