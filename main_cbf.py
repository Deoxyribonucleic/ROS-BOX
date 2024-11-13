import pyrealsense2 as rs
import numpy as np
import cv2

# Set up RealSense pipeline
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipe.start(cfg)

# Load YOLO model
net = cv2.dnn.readNet("/Users/erroraccount/Desktop/image/yolov3.weights", "/Users/erroraccount/Desktop/image/yolov3.cfg")
layer_names = net.getLayerNames()
output_layers = net.getUnconnectedOutLayers()
if isinstance(output_layers, np.ndarray):
    output_layers = output_layers.flatten()  
output_layers = [layer_names[i - 1] for i in output_layers]  

# Load class labels
with open("/Users/erroraccount/Desktop/image/coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]

try:
    while True:
        # Capture frames from RealSense
        frames = pipe.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # Convert RealSense frames to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Process depth image for visualization
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Object detection on the color image using YOLO
        blob = cv2.dnn.blobFromImage(color_image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        net.setInput(blob)
        detections = net.forward(output_layers)

        for out in detections:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]

                if confidence > 0.5:  # Filter out weak detections
                    center_x, center_y = int(detection[0] * color_image.shape[1]), int(detection[1] * color_image.shape[0])
                    w, h = int(detection[2] * color_image.shape[1]), int(detection[3] * color_image.shape[0])
                    x, y = int(center_x - w / 2), int(center_y - h / 2)

                    # Draw bounding box and label
                    cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(color_image, f"{classes[class_id]}: {confidence:.2f}", (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Display the RGB and depth images
        cv2.imshow("YOLO Object Detection", color_image)
        cv2.imshow("Depth Image", depth_colormap)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipe.stop()
    cv2.destroyAllWindows()
