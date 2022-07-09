# move_turtlebot3
Python script for moving turtlebot3 with semi closed loop control based on odometry feedback in simulation and real world.

**Code is derived from UGent's IDLab's Gabriel Urbain: https://gist.github.com/gurbain/c833e9858dd3e5fc4e30d6b1a305667b 

Directions (For Simulation):
- Run gazebo launch file on remote computer
`roslaunch turtlebot3_gazebo urtlebot3_empty_world.launch`
- Run python script on remote computer
`python move_turt`  

Directions (For Real Robots):
- Start ROS core on remote computer 
`roscore`
- Run bringup on turtlebot
`roslaunch turtlebot3_bringup turtlebot3_robot.launch`
- Run python script on remote computer
`python move_turt`  

*Note: Pass desired run_type parameter to MoveOdom object

Directions (For Multi-Robot Simulation):
- Run gazebo launch file on remote computer (i.e. example launch file is included in this repo)
`roslaunch turtlebot3_gazebo multi_turtlebot3_empty_world.launch`
- Run python script on remote computer
`python move_turt`  

Directions (For Multi-Real Robot):
- Start ROS core on remote computer 
`roscore`
- Run bringup on each turtlebot
`roslaunch turtlebot3_bringup turtlebot3_robot.launch`
*Note: Pass appropriate robot name onto each robot's ~/.bashrc by adding `export ROS_NAMESPACE= <Robot_Name>`
- Run python script on remote computer with robot_name specified
`python move_turt`  

