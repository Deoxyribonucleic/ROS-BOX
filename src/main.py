#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from math import atan2, sqrt, pi

#global var
x = 0.0
y = 0.0
theta = 0.0

# Box dimensions 
BOX_WIDTH = 0.5  # width erorr to adjust positions
BOX_LENGTH = 0.5  # 

def newOdom(msg):
    global x, y, theta
    x = msg.pose.pose.position.x  # X position of the robot
    y = msg.pose.pose.position.y  # Y position of the robot

    
    rot_q = msg.pose.pose.orientation
    orientation_list = [rot_q.x, rot_q.y, rot_q.z, rot_q.w]
    (roll, pitch, theta) = euler_from_quaternion(orientation_list)

#position of box (box_model)
def get_box_position():
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        # get service
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        # call service
        response = get_model_state("box_model", "")  # box_model

        # get pos of box
        box_position = response.pose.position
        return box_position.x, box_position.y
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return None, None

# move func
def move_to_goal(goal_x, goal_y):
    goal_tolerance = 0.2
    r = rospy.Rate(4)
    speed = Twist()

    while not rospy.is_shutdown():
        #distance
        inc_x = goal_x - x
        inc_y = goal_y - y
        distance_to_goal = sqrt(inc_x**2 + inc_y**2)

        # goal tolerance (error)
        if distance_to_goal <= goal_tolerance:
            #stop
            speed.linear.x = 0.0
            speed.angular.z = 0.0
            pub.publish(speed)
            rospy.loginfo("Reached the goal")
            return True

        #angle calc
        angle_to_goal = atan2(inc_y, inc_x)
        #angle diff
        angle_diff = angle_to_goal - theta

        # normalize the angle difference to the range (-pi, pi)
        if angle_diff > pi:
            angle_diff -= 2 * pi
        elif angle_diff < -pi:
            angle_diff += 2 * pi

        #adjst speed by angles
        if abs(angle_diff) > 0.1:
            speed.linear.x = 0.0
            # rotate clockwise or counterclockwise based on the shortest direction
            if angle_diff > 0:
                speed.angular.z = 0.3  # counterclockwise
            else:
                speed.angular.z = -0.3  # clockwise
        else:
            speed.linear.x = 0.5
            speed.angular.z = 0.0

        # publish velocity command
        pub.publish(speed)
        r.sleep()

    return False

# avoidance
def avoid_and_position(box_x, box_y):
    offset_distance = 0.5  #behind box tolerance
    rospy.loginfo("Calculating position to approach the box.")

#det box pos 
    if abs(box_x) > abs(box_y):
       #x axis
        target_x = box_x
        target_y = box_y + (offset_distance if box_y > 0 else -offset_distance)
    else:
        #y axis
        target_x = box_x + (offset_distance if box_x > 0 else -offset_distance)
        target_y = box_y

    rospy.loginfo(f"Moving to avoid box at ({target_x}, {target_y}).")
    move_to_goal(target_x, target_y)

# 
def move_box_to_origin(box_x, box_y):
    goal_tolerance = 0.2

    #avoid
    avoid_and_position(box_x, box_y)

    #
    target_x = box_x + (1.0 if box_x > 0 else -1.0)  # move further along x-axis
    target_y = box_y
    rospy.loginfo(f"Positioning robot further behind box at ({target_x}, {target_y}).")
    move_to_goal(target_x, target_y)

    # psh the box towards the origin
    rospy.loginfo("Pushing box to origin.")
    move_to_goal(0.0, 0.0)


rospy.init_node("box_to_origin")

sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

# Main 
while not rospy.is_shutdown():
    # get the current position of the box_model
    box_x, box_y = get_box_position()

    # skip this iteration if fail to get pos
    if box_x is None or box_y is None:
        rospy.logerr("Could not retrieve box position.")
        break

    # move the robot to the box position
    if move_to_goal(box_x, box_y):
        rospy.loginfo("Robot is at the box position.")
        move_box_to_origin(box_x, box_y)
        rospy.loginfo("Box has been moved to the origin.")
        break
    else:
        rospy.logwarn("Failed to reach box coordinates.")
        break
