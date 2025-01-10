#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt, pi

# Initialize global variables for position and orientation
x = 0.0
y = 0.0
theta = 0.0

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

# Initialize the ROS node
rospy.init_node("box_to_origin")

# Subscribers and Publishers
sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

# Twist message for velocity control
speed = Twist()

# Tolerance for reaching the goal
goal_tolerance = 0.2

# Set rate for the loop
r = rospy.Rate(4)

# Final goal position (origin)
final_goal_x = 0.0
final_goal_y = 0.0

# Main loop
while not rospy.is_shutdown():
    # Get the current position of the unit_box
    goal_x, goal_y = get_box_position()

    # If we failed to get the box position, skip this iteration
    if goal_x is None or goal_y is None:
        rospy.logerr("Could not retrieve box position.")
        break

    # Check if the box is at the origin
    if abs(goal_x - final_goal_x) <= goal_tolerance and abs(goal_y - final_goal_y) <= goal_tolerance:
        rospy.loginfo("Box is at the origin!")
        break

    # Determine the intermediate goal
    if abs(goal_x - final_goal_x) > goal_tolerance:
        intermediate_goal_x = final_goal_x
        intermediate_goal_y = goal_y
    else:
        intermediate_goal_x = goal_x
        intermediate_goal_y = final_goal_y

    # Calculate distance to intermediate goal
    inc_x = intermediate_goal_x - x
    inc_y = intermediate_goal_y - y
    distance_to_goal = sqrt(inc_x**2 + inc_y**2)

    # Check if the robot is within the goal tolerance
    if distance_to_goal <= goal_tolerance:
        # Stop the robot
        speed.linear.x = 0.0
        speed.angular.z = 0.0
        pub.publish(speed)
        rospy.loginfo("Intermediate goal reached!")
        continue

    # Calculate angle to intermediate goal
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

# Stop the robot
speed.linear.x = 0.0
speed.angular.z = 0.0
pub.publish(speed)
rospy.loginfo("Process complete.")
