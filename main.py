import cv2
import numpy as np
import pyrealsense2 as rs


# Set up RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()

# Configure the pipeline to stream color and depth frames
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start streaming
pipeline.start(config)

# Load YOLO model
net = cv2.dnn.readNet("/Users/erroraccount/Desktop/image/yolov3.weights", "/Users/erroraccount/Desktop/image/yolov3.cfg")
layer_names = net.getLayerNames()
output_layers = net.getUnconnectedOutLayers()

if isinstance(output_layers, np.ndarray):
    output_layers = output_layers.flatten()

output_layers = [layer_names[i - 1] for i in output_layers]

with open("/Users/erroraccount/Desktop/image/coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]

while True:
    # Wait for the next set of frames
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()

    if not color_frame:
        print("No frame captured")
        continue

    frame = np.asanyarray(color_frame.get_data())

    # YOLO object detection
    blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    detections = net.forward(output_layers)

    for out in detections:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]

            if confidence > 0.5:  
                center_x, center_y = int(detection[0] * frame.shape[1]), int(detection[1] * frame.shape[0])
                w, h = int(detection[2] * frame.shape[1]), int(detection[3] * frame.shape[0])
                x, y = int(center_x - w / 2), int(center_y - h / 2)

                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, f"{classes[class_id]}: {confidence:.2f}", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # Display the resulting frame
    cv2.imshow("YOLO Object Detection with RealSense", frame)

    if cv2.waitKey(1) & 0xFF == 27:  
        break

# Stop streaming
pipeline.stop()
cv2.destroyAllWindows()
