#! /usr/bin/env python

import random
import math
import time

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf import transformations

# global variables
state = 0 # robot state, which has 3 values, 0 - wandering, 1 - following wall, 2 - rotate
last_vel = [random.uniform(0.1,0.2),  random.uniform(-0.5,0.5)] # last wandering velocity

desired_wall_dist = 0.5 
detect_wall_distance = 1
actual_wall_dist = float("Inf")
rotate_speed = 15*2*3.14159/360
direction = 1 # direction used for rotate. -1 - right, 1 - left
wall_side = "unknown" # wall's relative position to robot. "left", "right", "front" and "unknown"

# Lidar data from /scan topic
front_area = []
left_area = []
right_area = []
front_left_area = []
back_left_area = []
front_right_area = []
back_right_area = []

# min distance at each range
min_front = float("Inf")
min_left = float("Inf")
min_right = float("Inf")
min_front_left = float("Inf")
min_front_right = float("Inf")
min_back_left = float("Inf")
min_back_right = float("Inf")


def cb_scan(msg):
    """
    call back function to deal with Lidar data, change robot state based on Lidar data and calculate wall distance
    """
    global front_area, left_area, right_area, front_left_area, back_left_area, front_right_area, back_right_area, min_front, min_left, min_right, min_back_left, min_back_right, min_front_left, min_front_right, state, actual_wall_dist, wall_side

    front_area = [x for x in (msg.ranges[0 : 22] + msg.ranges[338 : 359]) if x > 0 and x <= detect_wall_distance]
    left_area = [x for x in msg.ranges[68 : 112] if x > 0 and x <= detect_wall_distance]
    right_earea = [x for x in msg.ranges[248 : 292] if x > 0 and x <= detect_wall_distance]
    front_left_area = [x for x in msg.ranges[23 : 67] if x > 0 and x <= detect_wall_distance]
    front_right_area = [x for x in msg.ranges[293 : 337] if x > 0 and x <= detect_wall_distance]
    back_left_area = [x for x in msg.ranges[113 : 157] if x > 0 and x <= detect_wall_distance]
    back_right_area = [x for x in msg.ranges[203 : 247] if x > 0 and x <= detect_wall_distance]
    

    min_front = min(front_area) if len(front_area) > 0 else float("inf")
    min_left = min(left_area)if len(left_area) > 0 else float("inf")
    min_right = min(right_area)if len(right_area) > 0 else float("inf")
    min_front_left = min(front_left_area)if len(front_left_area) > 0 else float("inf")
    min_front_right = min(front_right_area)if len(front_right_area) > 0 else float("inf")
    min_back_left = min(back_left_area)if len(back_left_area) > 0 else float("inf")
    min_back_right = min(back_right_area)if len(back_right_area) > 0 else float("inf")
    

    if has_wall():
        actual_wall_dist = min(front_area + left_area + right_area + front_left_area + front_right_area + back_left_area + back_right_area)
        
        if min(min_left, min_front_left, min_back_left) == actual_wall_dist:
            wall_side = "left"
        elif min(min_right, min_front_right, min_back_right) == actual_wall_dist:
            wall_side = "right"
        elif min(front_area) == actual_wall_dist:
            wall_side = "front"
        else:
            wall_side = "unkown"

        if is_inner_corner() or is_outer_corner():
            state = 2
        else:
            state = 1
    else:
        state = 0

def has_wall():
    """
    use the data updated by scan callback function to determine if there is a wall around the robot
    """
    global front_area
    global left_area
    global right_area

    if len(front_right_area) > 0 or len(left_area) > 0 or len(right_area) > 0 or len(front_right_area) > 0 or len(front_left_area) > 0 or len(back_right_area) > 0 or len(back_left_area) > 0:
        return True
    return False

def is_inner_corner():
    """
    Determine whehter the wall around the robot is an inner corner and return result as a bool value
    """
    global direction, front_area, left_area, right_area

    if len(front_area) > 0 and len(left_area) > 0:
        direction = -1
        return True
    elif len(front_area) > 0 and len(right_area) > 0:
        direction = 1
        return True
    else:
        return False

def is_outer_corner():
    """
    Determine whether the wall around the robot is an outer corner and return result as a boo value
    """
    global front_area, left_area, right_area, front_left_area, back_left_area, front_right_area, back_right_area

    if len(front_area) == 0 and len(left_area) == 0 and len(right_area) == 0 and len(front_left_area) == 0 and len(front_right_area) == 0 and len(back_left_area) == 0 and len(back_right_area) > 0:
        direction = -1
        return True
    elif len(front_area) == 0 and len(left_area) == 0 and len(right_area) == 0 and len(front_left_area) == 0 and len(front_right_area) == 0 and len(back_left_area) > 0 and len(back_right_area) == 0:
        direction = 1
        return True
    else:
        return False

def rotate():
    """
    return a Twist velocity based on the direction global variable to make robot rotate 
    """
    global direction, rotate_speed

    t = Twist()
    t.angular.z = direction * 60*2*3.14159/360

    return t

def follow_wall():
    """
    try to follow the wall according to the wall's position relative to the robots and keep a distance to the wall
    """
    global desired_wall_dist
    global actual_wall_dist
    global wall_side
    global rotate_speed

    t = Twist()
    t.linear.x = 0.1

    if actual_wall_dist < desired_wall_dist:
        if wall_side == "left":
            t.angular.z = -rotate_speed
        else:
            t.angular.z = rotate_speed
    elif actual_wall_dist > desired_wall_dist:
        if wall_side == "left":
            t.angular.z = rotate_speed
        else:
            t.angular.z = -rotate_speed
    else:
        t.angular.z = 0

    return t


def wandering():
    """
    This function defines the linear.x and angular.z velocities for the random wandering of the robot.
    """
    global last_vel

    t = Twist()

    t.linear.x = min(max(last_vel[0] + random.uniform(-0.01,0.01), 0.1), 0.3)
    t.angular.z= min(max(last_vel[1] + random.uniform(-0.1,0.1), -1), 1)

    last_vel[0] = t.linear.x
    last_vel[1] = t.angular.z

    return t



def follow_wall_main():
    """
    This is the main function
    """
    global state

    rospy.init_node('follow_wall')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    sub = rospy.Subscriber('/scan', LaserScan, cb_scan)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        print(state)
        print(wall_side)
        print(actual_wall_dist)
        print("===========")

        t = Twist()

        if state == 0:
            t = wandering()
        elif state == 1:
            t = follow_wall()
        elif state == 2:
            t = rotate()
        else:
            t = t        
        pub.publish(t)

        rate.sleep()

if __name__ == "__main__":
    follow_wall_main()