#! /usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
import numpy as np

x = 0.0
y = 0.0
theta = 0.0

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion ([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node ("speed_controller")

sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel",Twist, queue_size=1)
speed = Twist()

r = rospy.Rate(4)

path_list = [(4,4), (3,1), (0,1), (-8,-8)]
point_index = 0  # instead of deleting stuff from a list (which is anyway bug prone) we'll just iterate through it using index variable.
goal = Point ()

while not rospy.is_shutdown():

     if point_index < len(path_list): # so we won't get an error of trying to reach non-existant index of a list
         goal.x = path_list[point_index][0] # x coordinate for goal
         goal.y = path_list[point_index][1] # y coordinate for goal
     else:
         break # I guess we're done?
     inc_x = goal.x - x
     inc_y = goal.y - y

     angle_to_goal = atan2 (inc_y, inc_x) # this is our "bearing to goal" as I can guess

     # distance_to_goal = np.sqrt(goal.x*goal.x + goal.y*goal.y) 
     2distance_to_goal = np.sqrt(inc_x*inc_x + inc_y*inc_y)

     if 2distance_to_goal >= 0.3: # we'll now head to our target
         if abs(angle_to_goal - theta) > 0.2:
             speed.linear.x = 0.0
             speed.angular.z = 0.9

         else:
             speed.linear.x = 0.8
             speed.angular.z = 0.0
         pub.publish(speed)

     else:
        point_index += 1
     r.sleep()
