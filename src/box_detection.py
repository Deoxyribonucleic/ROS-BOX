import pyrealsense2 as rs
import numpy as np
import cv2

# Set up RealSense pipeline
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipe.start(cfg)

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

        # Convert the color image to grayscale
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # Apply GaussianBlur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Use Canny edge detection
        edges = cv2.Canny(blurred, 50, 150)

        # Find contours in the edged image
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Iterate through contours and filter for rectangles
        for contour in contours:
            # Approximate the contour to a polygon
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # Check if the polygon has 4 vertices and is convex
            if len(approx) == 4 and cv2.isContourConvex(approx):
                # Calculate the bounding box and aspect ratio
                x, y, w, h = cv2.boundingRect(approx)
                aspect_ratio = w / float(h)

                # Filter based on aspect ratio and size
                if 0.8 < aspect_ratio < 1.2 and w * h > 5000:  # Adjust these thresholds as needed
                    # Check depth consistency
                    depth_roi = depth_image[y:y+h, x:x+w]
                    mean_depth = np.mean(depth_roi[depth_roi > 0])  # Exclude zero values
                    depth_std = np.std(depth_roi[depth_roi > 0])

                    # Ensure depth is consistent
                    if depth_std < 50:  # Adjust standard deviation threshold as needed
                        # Draw contours and bounding box
                        cv2.drawContours(color_image, [approx], -1, (0, 255, 0), 2)
                        cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
                        cv2.putText(
                            color_image, 
                            f"Depth: {mean_depth:.2f}", 
                            (x, y - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 
                            0.5, 
                            (255, 255, 255), 
                            1
                        )

        # Process depth image for visualization
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Display the RGB and depth images
        cv2.imshow("Detected Boxes", color_image)
        cv2.imshow("Depth Image", depth_colormap)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipe.stop()
    cv2.destroyAllWindows()
