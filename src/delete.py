#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from math import atan2, sqrt, pi

# Initialize global variables for position and orientation
x = 0.0
y = 0.0
theta = 0.0

# Box dimensions (adjust according to your simulation setup)
BOX_WIDTH = 0.5  # Width of the box
BOX_LENGTH = 0.5  # Length of the box

# Callback function to handle Odometry messages
def newOdom(msg):
    global x, y, theta
    x = msg.pose.pose.position.x  # X position of the robot
    y = msg.pose.pose.position.y  # Y position of the robot

    # Extract orientation from quaternion
    rot_q = msg.pose.pose.orientation
    orientation_list = [rot_q.x, rot_q.y, rot_q.z, rot_q.w]
    (roll, pitch, theta) = euler_from_quaternion(orientation_list)

# Function to get the position of the 'unit_box'
def get_box_position():
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        # Create a service proxy for the GetModelState service
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        # Call the service to get the state of 'unit_box'
        response = get_model_state("unit_box", "")

        # Extract the position of the box
        box_position = response.pose.position
        return box_position.x, box_position.y
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return None, None

# Function to move the robot to a given goal position
def move_to_goal(goal_x, goal_y):
    goal_tolerance = 0.2
    r = rospy.Rate(4)
    speed = Twist()

    while not rospy.is_shutdown():
        # Calculate distance to goal
        inc_x = goal_x - x
        inc_y = goal_y - y
        distance_to_goal = sqrt(inc_x**2 + inc_y**2)

        # Check if the robot is within the goal tolerance
        if distance_to_goal <= goal_tolerance:
            # Stop the robot
            speed.linear.x = 0.0
            speed.angular.z = 0.0
            pub.publish(speed)
            rospy.loginfo("Reached the goal coordinates!")
            return True

        # Calculate angle to goal
        angle_to_goal = atan2(inc_y, inc_x)

        # Calculate the difference between the current orientation and the angle to the goal
        angle_diff = angle_to_goal - theta

        # Normalize the angle difference to the range (-pi, pi)
        if angle_diff > pi:
            angle_diff -= 2 * pi
        elif angle_diff < -pi:
            angle_diff += 2 * pi

        # Adjust speed based on angle difference
        if abs(angle_diff) > 0.1:
            speed.linear.x = 0.0
            # Rotate clockwise or counterclockwise based on the shortest direction
            if angle_diff > 0:
                speed.angular.z = 0.3  # Counterclockwise
            else:
                speed.angular.z = -0.3  # Clockwise
        else:
            speed.linear.x = 0.5
            speed.angular.z = 0.0

        # Publish the velocity command
        pub.publish(speed)
        r.sleep()

    return False

# Function to push the box along one axis towards the origin
def push_box_to_axis(box_x, box_y):
    # Determine which axis to align first
    goal_tolerance = 0.2

    if abs(box_x) > goal_tolerance:
        # Move to a position behind the box to push it along the x-axis
        target_x = box_x + (1.0 if box_x > 0 else -1.0)
        target_y = box_y
        rospy.loginfo("Moving to push position for x-axis alignment.")
        move_to_goal(target_x, target_y)

        # Rotate towards the origin
        angle_to_origin = atan2(0 - box_y, 0 - box_x)
        rotate_to_angle(angle_to_origin)

        # Push the box towards the origin along the x-axis
        rospy.loginfo("Pushing box along x-axis towards the origin.")
        move_to_goal(0.0, box_y)

    if abs(box_y) > goal_tolerance:
        # Move to a position behind the box to push it along the y-axis
        target_x = 0.0
        target_y = box_y + (1.0 if box_y > 0 else -1.0)
        rospy.loginfo("Moving to push position for y-axis alignment.")
        move_to_goal(target_x, target_y)

        # Rotate towards the origin
        angle_to_origin = atan2(0 - box_y, 0 - box_x)
        rotate_to_angle(angle_to_origin)

        # Push the box towards the origin along the y-axis
        rospy.loginfo("Pushing box along y-axis towards the origin.")
        move_to_goal(0.0, 0.0)

# Function to rotate the robot to a specific angle
def rotate_to_angle(target_angle):
    r = rospy.Rate(10)
    speed = Twist()

    while not rospy.is_shutdown():
        angle_diff = target_angle - theta

        # Normalize the angle difference
        if angle_diff > pi:
            angle_diff -= 2 * pi
        elif angle_diff < -pi:
            angle_diff += 2 * pi

        if abs(angle_diff) < 0.1:
            speed.angular.z = 0.0
            pub.publish(speed)
            rospy.loginfo("Rotation complete.")
            return

        speed.angular.z = 0.3 if angle_diff > 0 else -0.3
        pub.publish(speed)
        r.sleep()

# Initialize the ROS node
rospy.init_node("box_to_origin")

# Subscribers and Publishers
sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

# Main logic
while not rospy.is_shutdown():
    # Get the current position of the unit_box
    box_x, box_y = get_box_position()

    # If we failed to get the box position, skip this iteration
    if box_x is None or box_y is None:
        rospy.logerr("Could not retrieve box position.")
        break

    # Move the robot to the box position + offset and push towards the origin
    rospy.loginfo("Starting push sequence.")
    push_box_to_axis(box_x, box_y)
    rospy.loginfo("Box has been moved to the origin.")
    break
