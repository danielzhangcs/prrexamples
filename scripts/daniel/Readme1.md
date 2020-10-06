# Teach a robot to dance
author -- Daniel Zhang (danielzhang@brandeis.edu)

## Project Introduction
This project is about teaching robot to dance. It has two parts. one is key_publiser which is responsible for listening on the input from keyboard and publish the input though a topic called "keys". 

Then another part of this project is responsible for subscribing to key input from keyboard and make corresponding movement. In this part, specifically, once the subscriber received message from "\keys" topic, it will call the callback function to update the state. Then the loop will detect the update of state and make movements.

## How to start
In order to use this project we need to enter the following commands to start roscore, launch gazebo and start related nodes to receive and publish message. 
* $ roscore  
* $ roslaunch turtlebot3_stage_1.launch  
* $ rosrun key_publisher.py
* $ rosrun DanielZhang_PA4.py

## Available commands
l: rotate left
r: rotate right
f: move forward
b: move backward
h: halt all motion
s: spiraling motion
z: zigzag motion
w: Waltz
