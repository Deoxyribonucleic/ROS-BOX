import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

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

# Initialize the ROS node
rospy.init_node("speed_controller")

# Subscribers and Publishers
sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

# Twist message for velocity control
speed = Twist()

# Set goal point
goal = Point()
goal.x = 5
goal.y = 5

# Set rate for the loop
r = rospy.Rate(4)

# Main loop
while not rospy.is_shutdown():
    # Calculate distance to goal
    inc_x = goal.x - x
    inc_y = goal.y - y

    # Calculate angle to goal
    angle_to_goal = atan2(inc_y, inc_x)

    # Adjust speed based on angle difference
    if abs(angle_to_goal - theta) > 0.1:
        speed.linear.x = 0.0
        speed.angular.z = 0.3
    else:
        speed.linear.x = 0.5
        speed.angular.z = 0.0

    # Publish the velocity command
    pub.publish(speed)
    r.sleep()
