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

# Initialize the ROS node
rospy.init_node("box_to_origin")

# Subscribers and Publishers
sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

# Main logic
while not rospy.is_shutdown():
    # Get the current position of the unit_box
    goal_x, goal_y = get_box_position()

    # If we failed to get the box position, skip this iteration
    if goal_x is None or goal_y is None:
        rospy.logerr("Could not retrieve box position.")
        break

    # Move the robot to the box position
    if move_to_goal(goal_x, goal_y):
        rospy.loginfo("Robot is at the box position.")
        # Rest of the code to push the box goes here
        # Determine intermediate goals and move box to origin as previously defined

        # Check if the box is at the origin
        final_goal_x = 0.0
        final_goal_y = 0.0
        goal_tolerance = 0.2

        while not rospy.is_shutdown():
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
                rospy.loginfo("Intermediate goal reached!")
                break

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
            speed = Twist()
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
            rospy.Rate(4).sleep()

        rospy.loginfo("Box has been moved to the origin.")
        break
    else:
        rospy.logwarn("Failed to reach box coordinates.")
        break
