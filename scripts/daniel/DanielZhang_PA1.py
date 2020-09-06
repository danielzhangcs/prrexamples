#!/usr/bin/env python

import time
import math

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

PI = 3.1415926535897

def move(publisher, distance):
    twist = Twist()
    speed = 0.1

    #set move parameters
    twist.linear.x = 0.1
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0

    #set curretn time and distance for caculation
    current_distance = 0
    t0 = time.time()

    while not rospy.is_shutdown():
        
        while current_distance < distance:
            publisher.publish(twist)
            t1 = time.time()
            current_distance = speed * (t1 - t0)
        
        #Force robot to stop move
        twist.linear.x = 0
        publisher.publish(twist)
        
        return current_distance

def rotate(publisher, angle):
    twist = Twist()
    speed = 100

    #Converting from angles to radianss
    angular_speed = speed*2*PI/360
    relative_angle = angle*2*PI/360

    #set twist parameters
    twist.linear.x=0
    twist.linear.y=0
    twist.linear.z=0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = abs(angular_speed)

    # Setting current time and current angle for calculation
    t0 = time.time()
    current_angle = 0

    while not rospy.is_shutdown():

        while current_angle < relative_angle:
            publisher.publish(twist)
            t1 = time.time()
            current_angle = angular_speed * (t1-t0)

        #Forcing our robot to stop rotate
        twist.angular.z = 0
        publisher.publish(twist)

        return current_angle

if __name__ == "__main__":

    # Make this into a ROS node.
    rospy.init_node('topic_publisher')

    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    distance = move(publisher, 1.5)

    angle = rotate(publisher, 180)
  
    move(publisher, distance)

