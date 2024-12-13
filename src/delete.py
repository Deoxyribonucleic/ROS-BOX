#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def move_forward():
    # Initialize the ROS node
    rospy.init_node('move_robot_forward', anonymous=True)

    # Create a publisher to the 'cmd_vel' topic with message type Twist
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Set the rate of publishing commands (10 Hz)
    rate = rospy.Rate(10)

    # Create a Twist message to store velocity commands
    move_cmd = Twist()

    # Set linear velocity (move forward along the x-axis) and angular velocity (no turning)
    move_cmd.linear.x = 0.5  # Forward speed (m/s)
    move_cmd.angular.z = 0.0  # No rotation

    # Print a message and publish the velocity for 5 seconds
    rospy.loginfo("Moving robot forward at 0.5 m/s")
    duration = 5  # Move for 5 seconds

    start_time = rospy.get_time()
    while rospy.get_time() - start_time < duration and not rospy.is_shutdown():
        pub.publish(move_cmd)  # Publish the move command
        rate.sleep()  # Sleep to maintain 10 Hz

    # Stop the robot after 5 seconds
    move_cmd.linear.x = 0.0
    pub.publish(move_cmd)
    rospy.loginfo("Stopping robot")

if __name__ == '__main__':
    try:
        move_forward()
    except rospy.ROSInterruptException:
        pass
