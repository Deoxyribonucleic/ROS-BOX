import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, pi

# Global variables
x = 0.0
y = 0.0
theta = 0.0

# Callback function for odometry updates
def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

# Function to normalize angles to [-pi, pi]
def normalize_angle(angle):
    while angle > pi:
        angle -= 2 * pi
    while angle < -pi:
        angle += 2 * pi
    return angle

# Initialize ROS node
rospy.init_node("speed_controller")

# Subscriber and Publisher
sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

# Speed command
speed = Twist()

# Rate of the loop
r = rospy.Rate(4)

# Goal point
goal = Point()
goal.x = 5
goal.y = 5

# Main loop
while not rospy.is_shutdown():
    # Calculate increments and angle to goal
    inc_x = goal.x - x
    inc_y = goal.y - y

    angle_to_goal = atan2(inc_y, inc_x)
    angular_error = normalize_angle(angle_to_goal - theta)

    # Control logic
    if abs(angular_error) > 0.1:  # Angular alignment
        speed.linear.x = 0.0
        speed.angular.z = 0.5 * angular_error  # Proportional control
    else:  # Move towards the goal
        speed.linear.x = 0.5
        speed.angular.z = 0.0

    # Publish speed command
    pub.publish(speed)
    r.sleep()
