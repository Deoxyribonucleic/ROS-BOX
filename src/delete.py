#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import Twist

def move_robot(direction, duration=2.0):
    # Initialize the ROS node
    rospy.init_node('move_robot', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # Set the movement parameters
    move_cmd = Twist()
    if direction == 'forward':
        move_cmd.linear.x = 0.5  # Move forward
    elif direction == 'backward':
        move_cmd.linear.x = -0.5  # Move backward
    elif direction == 'left':
        move_cmd.angular.z = 0.5  # Turn left
    elif direction == 'right':
        move_cmd.angular.z = -0.5  # Turn right
    else:
        rospy.loginfo("Invalid direction. Use 'forward', 'backward', 'left', or 'right'.")
        return

    # Move the robot for the specified duration
    start_time = rospy.Time.now()
    while rospy.Time.now() - start_time < rospy.Duration(duration):
        pub.publish(move_cmd)
        rate.sleep()

    # Stop the robot after moving
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)
    rospy.loginfo("Movement complete.")

if __name__ == '__main__':
    try:
        if len(sys.argv) < 2:
            print("Usage: rosrun <package_name> move_robot.py <direction> [duration]")
            sys.exit(1)
        
        direction = sys.argv[1].strip().lower()
        duration = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0

        move_robot(direction, duration)
    except rospy.ROSInterruptException:
        pass
    except ValueError:
        print("Invalid duration. Please provide a numerical value for duration.")
