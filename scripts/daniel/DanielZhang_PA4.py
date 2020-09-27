#!/usr/bin/env python

import rospy
import sys
import math
import tf
import time
import random

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

key_mapping = { 
    'f': [0, 0.2], 'b': [0, -0.2], 
    'l': [30*2*math.pi/360, 0], 'r': [-30*2*math.pi/360,  0],
    'h':[0,0], 's': [180*2*math.pi/360, 0.3],
    'z0': [0, 0.3], 'z1':[-65*2*math.pi/360, 0],
    'z2':[65*2*math.pi/360, 0]
}

pahse_mapping = {
    0: '0',
    1: '0',
    2: '1',
    3: '1',
    4: '0',
    5: '0',
    6: '2',
    7: '2'
}

front_area = []
back_area = []
current_position = [0, 0]
current_speed = Twist()

# fill in scan callback
def scan_cb(msg):
    global front_area
    global back_area
    global state

    front_area = [x for x in (msg.ranges[0 : 45] + msg.ranges[315 : 359]) if x > 0 and x < 0.2]
    back_area = [x for x in msg.ranges[135 : 224] if x > 0 and x < 0.25]

    if len(front_area) > 0:
        if state == 'f' or state == 's' or state == 'z' or state == 'w':
            state = "h"

    if len(back_area) > 0:
        if state == 'b':
            state = "h"
    
# it is not necessary to add more code here but it could be useful
def key_cb(msg):
   global state
   state = msg.data
   last_key_press_time = time.time()

# odom is also not necessary but very useful
def odom_cb(msg):
   global current_position
   global current_speed
   current_position[0] = msg.pose.pose.position.x
   current_position[1] = msg.pose.pose.position.y

   current_speed = msg.twist.twist


# print the state of the robot
def print_state():
    print("---")
    print("STATE: " + state)
    
    # calculate time since last key stroke
    time_since = time.time() - last_key_press_time
    
    print("SECS SINCE LAST KEY PRESS: " + str(time_since))
    print("Current Location: x ({}), y ({})".format(current_position[0], current_position[1]))
    print("Current Speed: {}".format(current_speed))

# init node
rospy.init_node('dancer')

# subscribers/publishers
scan_sub = rospy.Subscriber('scan', LaserScan, scan_cb)

# RUN rosrun prrexamples key_publisher.py to get /keys
key_sub = rospy.Subscriber('keys', String, key_cb)
odom_sub = rospy.Subscriber('odom', Odometry, odom_cb)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

# start in state halted and grab the current time
state = "h"
last_key_press_time = time.time()

# set rate
rate = rospy.Rate(10)

# Wait for published topics, exit on ^c
while not rospy.is_shutdown():
    # print out the current state and time since last key press
    print_state()
    
    # publish cmd_vel from here 
    t = Twist()
    
    if state == 'z':
       time_slot = int((time.time() - last_key_press_time)  % 8)
       phase = pahse_mapping[time_slot]
       t.linear.x = key_mapping[state + phase][1]
       t.angular.z = key_mapping[state + phase][0]

    elif state == 's':
        t.linear.x = key_mapping[state][1]
        t.angular.z = max(0.3, key_mapping[state][0] - ((time.time() - last_key_press_time)  / 20))

    elif state == 'w':
        phase = int((time.time() - last_key_press_time)  % 10)
        if phase < 5:
            t.linear.x = 0.3
            t.angular.z = 0
        else:
            t.linear.x = 0
            t.angular.z = 360*2*math.pi/360
        

    elif state not in key_mapping:
       continue

    else :
        t.linear.x = key_mapping[state][1]
        t.angular.z = key_mapping[state][0] 
    
    cmd_vel_pub.publish(t)
    
    rate.sleep()