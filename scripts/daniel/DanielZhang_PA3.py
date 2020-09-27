#!/usr/bin/env python

import math
import time
import random

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


# Function is called every time there's a scan
# Saves the smallest distance, which would be the closest obstacle
# This is without regard to the direction of the obstacle!
# global keyword is needed so g_range_ahead is available outside of the function
def scan_callback(msg):
    global filtered_ranges
    filtered_ranges = [x for x in (msg.ranges[0 : 30] + msg.ranges[330 : 359]) if x > 0 and x < 0.2 ]
    

# Main program
filtered_ranges = [] # anything to start
rotate_speed = 30*2*math.pi/360

# Declare a subscriber to message 'scan' with message class LaserScan
scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)

# Same code can be a publisher and a subscriber, this is no problem
# be ready to publish the cmd_vel topic of class Twist
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

# Declare us to be a node
rospy.init_node('wander')
state_change_time = rospy.Time.now()

# driving_forward: forward(true) vs. spin inplace (false)
#   TRUE: until x seconds pass or get close to an obstacle, go forward
#   FALSE: until y seconds pass, spin in place
driving_forward = True
rate = rospy.Rate(20)
twist = Twist()

while not rospy.is_shutdown():

    # check whether antyhing is closer than x meters or 
    # time for driving foreward is over, then start spinning in place
    print("============")
    print(filtered_ranges)
    print("==============")

    if len(filtered_ranges) == 0:
        twist.linear.x = 0.1
        twist.angular.z = 0
        cmd_vel_pub.publish(twist)

        rate.sleep()
    
    else:
        current_angle = 0
        t0 = time.time()
        rotate_angle = random.randint(15, 45)*2*math.pi/360
        while current_angle < rotate_angle:
                twist.linear.x = 0
                twist.angular.z = rotate_speed
                cmd_vel_pub.publish(twist)
                t1 = time.time()
                current_angle = rotate_speed * (t1-t0)

                rate.sleep()
    
    rate.sleep()

