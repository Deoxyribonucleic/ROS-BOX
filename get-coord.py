import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers

marker_data = {}

def callback(data):
    global marker_data
    for marker in data.markers:
        marker_data[marker.id] = {
            "position": (
                marker.pose.pose.position.x,
                marker.pose.pose.position.y,
                marker.pose.pose.position.z
            ),
            "orientation": (
                marker.pose.pose.orientation.x,
                marker.pose.pose.orientation.y,
                marker.pose.pose.orientation.z,
                marker.pose.pose.orientation.w
            )
        }

def print_coordinates(event):
    global marker_data
    if marker_data:
        rospy.loginfo("Updated Marker Coordinates:")
        for marker_id, coords in marker_data.items():
            rospy.loginfo(f"Marker ID: {marker_id}")
            rospy.loginfo(f"  Position: {coords['position']}")
            rospy.loginfo(f"  Orientation: {coords['orientation']}")
    else:
        rospy.loginfo("No markers detected.")

rospy.init_node('ar_marker_listener', anonymous=True)
rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback)

# Set up a timer to call print_coordinates every 5 seconds
timer = rospy.Timer(rospy.Duration(5), print_coordinates)

rospy.spin()
