#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy
from tf.transformations import euler_from_quaternion

robot_pose = None  # Global variable to store robot's current pose

def odom_callback(data):
    """Callback function to update the robot's pose."""
    global robot_pose
    robot_pose = data.pose.pose

def move_forward():
    # Initialize the ROS node
    rospy.init_node('move_robot_forward', anonymous=True)
    rospy.Subscriber('/odom', Odometry, odom_callback)  # Subscribe to odometry data
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    goal_x = 5.0
    goal_y = 5.0
    position_tolerance = 0.1  # Tolerance for stopping near the goal

    # Wait until the robot's pose is available
    global robot_pose
    while robot_pose is None and not rospy.is_shutdown():
        rospy.loginfo("Waiting for robot pose...")
        rospy.sleep(0.1)

    rate = rospy.Rate(10)

    # Extract the initial robot pose and orientation
    robot_x = robot_pose.position.x
    robot_y = robot_pose.position.y
    orientation_q = robot_pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    _, _, robot_yaw = euler_from_quaternion(orientation_list)

    # Calculate the angle to the goal
    angle_to_goal = numpy.arctan2(goal_y - robot_y, goal_x - robot_x)

    # Calculate the angular difference and normalize it
    angular_difference = angle_to_goal - robot_yaw
    angular_difference = numpy.arctan2(numpy.sin(angular_difference), numpy.cos(angular_difference))  # Normalize

    # Rotate the robot to face the goal
    move_cmd = Twist()
    move_cmd.angular.z = 0.5 if angular_difference > 0 else -0.5  # Rotate direction
    rospy.loginfo("Rotating robot to face the goal...")
    while abs(angular_difference) > 0.1 and not rospy.is_shutdown():
        pub.publish(move_cmd)
        rate.sleep()

        # Update the angular difference
        robot_x = robot_pose.position.x
        robot_y = robot_pose.position.y
        orientation_q = robot_pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, robot_yaw = euler_from_quaternion(orientation_list)
        angular_difference = angle_to_goal - robot_yaw
        angular_difference = numpy.arctan2(numpy.sin(angular_difference), numpy.cos(angular_difference))

    # Stop rotation
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)

    # Move forward toward the goal
    move_cmd.linear.x = 0.5  # Forward speed (m/s)
    rospy.loginfo("Moving robot forward at 0.5 m/s")
    while not rospy.is_shutdown():
        robot_x = robot_pose.position.x
        robot_y = robot_pose.position.y

        # Check if the robot is near the goal position
        if abs(goal_x - robot_x) < position_tolerance and abs(goal_y - robot_y) < position_tolerance:
            rospy.loginfo("Goal reached! Stopping robot.")
            break

        pub.publish(move_cmd)
        rate.sleep()

    # Stop the robot
    move_cmd.linear.x = 0.0
    pub.publish(move_cmd)
    rospy.loginfo("Stopping robot")

if __name__ == '__main__':
    try:
        move_forward()
    except rospy.ROSInterruptException:
        pass
