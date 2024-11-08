import rospy
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers

def get_robot_position(N):
    # This function gets the position of AR tags for N robots
    AlvarMsg = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)
    pose = np.empty((4, N), float)

    while True:
        if len(AlvarMsg.markers) == N:
            for m in AlvarMsg.markers:
                pose[0, m.id] = m.pose.pose.position.y  # Adjusted for camera frame
                pose[1, m.id] = -m.pose.pose.position.z
                orientation_q = m.pose.pose.orientation
                orientation_list = [orientation_q.y, orientation_q.z, orientation_q.x, orientation_q.w]
                (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
                pose[2, m.id] = -yaw
                pose[3, m.id] = m.id
            break
        else:
            print("Waiting for markers...")
            AlvarMsg = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)

    ind = np.argsort(pose[3, :])
    pose = pose[:, ind]
    return pose[[0, 1, 2], :]

def put_velocities(N, dxu):
    # Publishes velocity commands to each robot
    for i in range(N):
        velPub = rospy.Publisher(f'raspi_{i}/cmd_vel', Twist, queue_size=3)
        velMsg = create_vel_msg(dxu[0, i], dxu[1, i])
        velPub.publish(velMsg)

def create_vel_msg(v, w):
    # Creates a Twist message for robot movement
    velMsg = Twist()
    velMsg.linear.x = v
    velMsg.linear.y = 0
    velMsg.linear.z = 0
    velMsg.angular.x = 0
    velMsg.angular.y = 0
    velMsg.angular.z = w
    return velMsg

def set_velocities(ids, velocities, max_linear_velocity=0.3, max_angular_velocity=6):
    # Adjusts velocities within max linear and angular constraints
    idxs = np.where(np.abs(velocities[0, :]) > max_linear_velocity)
    velocities[0, idxs] = max_linear_velocity * np.sign(velocities[0, idxs])

    idxs = np.where(np.abs(velocities[1, :]) > max_angular_velocity)
    velocities[1, idxs] = max_angular_velocity * np.sign(velocities[1, idxs])
    return velocities

def main():
    rospy.init_node('robot_control')
    N = 3  # Example: Number of robots

    while not rospy.is_shutdown():
        # Get robot positions based on AR tags
        positions = get_robot_position(N)

        # Example control strategy (could be any control algorithm)
        dxu = np.zeros((2, N))
        for i in range(N):
            # Set desired velocity for each robot (modify as needed)
            dxu[0, i] = 0.2  # Example linear velocity
            dxu[1, i] = 0.1  # Example angular velocity

        # Set the velocities with constraints
        constrained_velocities = set_velocities(range(N), dxu)

        # Send velocities to each robot
        put_velocities(N, constrained_velocities)

if __name__ == "__main__":
    main()
