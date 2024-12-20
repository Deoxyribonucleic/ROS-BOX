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

    goal_x = 2.0
    goal_y = 2.0
    position_tolerance =   # Tolerance for stopping near the goal

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

    move_cmd = Twist()

    # Move towards the x-axis goal first
    rospy.loginfo("Moving robot towards X-axis goal")

    while not rospy.is_shutdown():
        robot_x = robot_pose.position.x
        robot_y = robot_pose.position.y

        # Check if the robot has reached the goal on the x-axis
        if abs(goal_x - robot_x) < position_tolerance:
            rospy.loginfo(f"Goal reached on X-axis: {robot_x}")
            # If x-axis goal is reached, stop moving along x-axis
            move_cmd.linear.x = 0.0
            pub.publish(move_cmd)

            # Now rotate robot to face the y-axis (0 degree yaw)
            rospy.loginfo("Rotating robot to face Y-axis (0 degree yaw)")
            target_yaw = 0.0
            # Calculate the angular difference and normalize it
            angular_difference = target_yaw - robot_yaw
            angular_difference = numpy.arctan2(numpy.sin(angular_difference), numpy.cos(angular_difference))  # Normalize

            move_cmd.angular.z = 0.5 if angular_difference > 0 else -0.5  # Rotate direction
            while abs(angular_difference) > 0.1 and not rospy.is_shutdown():
                pub.publish(move_cmd)
                rate.sleep()

                # Update the angular difference after rotation
                robot_x = robot_pose.position.x
                robot_y = robot_pose.position.y
                orientation_q = robot_pose.orientation
                orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
                _, _, robot_yaw = euler_from_quaternion(orientation_list)
                angular_difference = target_yaw - robot_yaw
                angular_difference = numpy.arctan2(numpy.sin(angular_difference), numpy.cos(angular_difference))

            # Stop rotation
            move_cmd.angular.z = 0.0
            pub.publish(move_cmd)

            # Start moving towards the y-axis goal
            rospy.loginfo("Moving robot towards Y-axis goal")
            while not rospy.is_shutdown():
                robot_x = robot_pose.position.x
                robot_y = robot_pose.position.y

                # Move along the y-axis
                if abs(goal_y - robot_y) < position_tolerance:
                    rospy.loginfo(f"Goal reached on Y-axis: {robot_y}")
                    move_cmd.linear.y = 0.0  # Stop moving along y-axis
                    break
                else:
                    move_cmd.linear.y = 0.5  # Continue moving along y-axis

                pub.publish(move_cmd)
                rate.sleep()

            # Stop the robot after reaching the y-axis goal
            move_cmd.linear.y = 0.0
            pub.publish(move_cmd)
            rospy.loginfo("Goal reached on Y-axis. Stopping robot.")
            break
        else:
            move_cmd.linear.x = 0.5  # Continue moving along x-axis
            pub.publish(move_cmd)

        rate.sleep()

    # Final stop command to ensure the robot halts
    move_cmd.linear.x = 0.0
    pub.publish(move_cmd)
    rospy.loginfo("Stopping robot.")

if __name__ == '__main__':
    try:
        move_forward()
    except rospy.ROSInterruptException:
        pass
