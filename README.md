# Homework_2_RL

 
## Running Gazebo and Rviz
To spawn the robot in Gazebo and Rviz with the velocity controller:
```bash
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller"
```
The arguments of `iiwa.launch.py` are:
- `use_sim` (default: "false") - Start robot in Gazebo simulation.
- `command_interface` (default: "position") - Robot command interface [position|velocity|effort].
- `robot_controller` (default: "iiwa_arm_controller" for the position_controller) - Robot controller to start.
 
To spawn the robot in Gazebo and Rviz with the effort controller:
```bash
ros2 launch iiwa_bringup iiwa.launch.py use_sim:="true" command_interface:="effort" robot_controller:="effort_controller"
```
## IMPORTANT
When gazebo world is open, the simulation is in pause and with zero gravity. Then we click on the small play button in the bottom left corner of the GUI in order to start the simulation.
 
## Running ros2_kdl_node.cpp
To run the executable file use the command:
```bash
ros2 run ros2_kdl_package ros2_kdl_node
```
The arguments of `ros2_kdl_node` are:
- `cmd_interface` (default: "position") - [position|velocity|effort].
- `trajectory_type` (default: "trapezoidal") - to choose the velocity profile [trapezoidal|cubic].
 
To control the robot in velocity:
```bash
ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=velocity
```
To control the robot in effort:
```bash
ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=effort
```
In order to choose the trajectory type we defined a radius in the file ros2_kdl_node.cpp.
If the radius is greater than 0 returns a circular trajectory, otherwise the linear one.
To choose the velocity profile, for example: 
```bash
ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=velocity -p trajectory_type:=cubic
```
## Inverse Dynamics Controllers
In the effort controll to choose which controller to use ,after launching the executable ros2_kdl_node.cpp, in the terminal we can send 0 or 1, corresponding to the Joint Space Inverse Dynamics Controller and the Inverse Dynamics Operational Space Controller.
 
## Plot
To plot the joint torques, run in an other terminal:
```bash
rqt
```
then go in `Plugins->Visualization->Plot` and add `/effort_controller/commands/data[0]` up to `/effort_controller/commands/data[6]`.
