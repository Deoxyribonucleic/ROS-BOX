import rospy
import numpy as np
import cv2
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers

# Load YOLO model
net = cv2.dnn.readNet("/Users/erroraccount/Desktop/image/yolov3.weights", "/Users/erroraccount/Desktop/image/yolov3.cfg")
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

with open("/Users/erroraccount/Desktop/image/coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]

# Set up video capture (USB camera, camera ID may vary)
cap = cv2.VideoCapture(0)

def get_robot_position(N):
    # Similar to the original code to get AR tag positions for N robots
    AlvarMsg = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)
    pose = np.empty((4, N), float)

    while True:
        if len(AlvarMsg.markers) == N:
            for m in AlvarMsg.markers:
                pose[0, m.id] = m.pose.pose.position.y  # Adjust as per camera frame
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

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # YOLO object detection
    blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    detections = net.forward(output_layers)

    for out in detections:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]

            if confidence > 0.5:  # Confidence threshold
                center_x, center_y = int(detection[0] * frame.shape[1]), int(detection[1] * frame.shape[0])
                w, h = int(detection[2] * frame.shape[1]), int(detection[3] * frame.shape[0])
                x, y = int(center_x - w / 2), int(center_y - h / 2)

                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, f"{classes[class_id]}: {confidence:.2f}", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # Display the result with detections
    cv2.imshow("YOLO Object Detection with AR Tag Localization", frame)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
