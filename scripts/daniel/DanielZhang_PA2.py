#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
import math

# global variables
x = 0.0
theta = 0.0
speed = Twist()
r = rospy.Rate(10)
goal = Point()
finish = False

# call back function
def callback(msg):
    global x
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation

    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

#init node as "daniel_node"
rospy.init_node("daniel_node")

# subscribe to odom and trigger the call back function to update the global variables(robot location)
sub = rospy.Subscriber("/odom", Odometry, callback)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)


while not rospy.is_shutdown():

    goal.x = 1.5
    inc_x = goal.x - x

    # move 1.5 forward
    while inc_x > 0.001 and not finish:
        speed.linear.x = 0.1
        pub.publish(speed)
        r.sleep()   
        inc_x = goal.x - x
    
    # turn around
    while abs(theta - math.pi) > 0.1 and not finish:
        speed.linear.x = 0.0
        speed.angular.z = 0.3
        pub.publish(speed)
        r.sleep()  
    

    # set origin point as goal and move back
    goal.x = 0
    inc_x = x - goal.x
    while inc_x > 0.001 and not finish:
        speed.linear.x = 0.1
        speed.angular.z = 0
        pub.publish(speed)
        r.sleep()   

        inc_x = x - goal.x

    # task finish, publish stop velocity
    finish = True
    speed.linear.x=0
    pub.publish(speed)
    r.sleep()
        


    


    
    

