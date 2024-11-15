import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers

def callback(data):
    for marker in data.markers:
        print(f"Marker ID: {marker.id}")
        print(f"Position (x, y, z): ({marker.pose.pose.position.x}, {marker.pose.pose.position.y}, {marker.pose.pose.position.z})")
        print(f"Orientation (x, y, z, w): ({marker.pose.pose.orientation.x}, {marker.pose.pose.orientation.y}, {marker.pose.pose.orientation.z}, {marker.pose.pose.orientation.w})")

rospy.init_node('ar_marker_listener', anonymous=True)
rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback)
rospy.spin()
