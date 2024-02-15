# gazebo_docking_simulation
This repository contain gazebo simulation files and python codes required for a patroling robot which automatically dock when battery percentage is less than 30 percentage.It is assumed that when robot travels a perticular distance (obtained from encoder feedback) battery would be at 30 percentage. It utilizes 3 fiducial markers to locate the position of the docking station. Hence, the Robot should equip with a camera input for fiducial marker detection.

Steps
1.setup turtlebot simulation packages 
Refer : https://automaticaddison.com/how-to-launch-the-turtlebot3-simulation-with-ros/
2.Run gazebo 
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
3.Run Rviz
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$path/to/map/file/map.yaml
Localize robot with help of 2D pose estimate arrow key
4.Run distance node
rosrun turtlebot3_navigation distance.py
5.Run nav python file 
rosrun turtlebot3_navigation nav.py
This python file enable robot to autonomously patrol and command robot to go to charging dock position if battery charge is less than 30 percentage.
6.Autonomous docking command
rostopic pub /autodock_action/goal autodock_core/AutoDockingActionGoal {} --once
