# Teach a robot to dance
author -- Daniel Zhang (danielzhang@brandeis.edu)

## Project Introduction
This project is about making robot to follow a wall. 

The robot will firstly randomly wandoring if there are no wall.

Once the Lidar data shows that there is a wall around the robot, it will start to follow it and keep a distance at 0.5 meters. it will keep adjusting linear speed and anguar speed to maintain the distance to the wall.

If there is a corner, it will use the Lidar data to determine it is an ourter corner or inner corner and make rorate accordingly. 

## How to start
In order to use this project we need to enter the following commands to start roscore, launch gazebo and start related nodes to receive and publish message. 
* $ roscore  
* $ roslaunch turtlebot3_stage_1.launch  
* $ rosrun DanielZhang_PA4.py
