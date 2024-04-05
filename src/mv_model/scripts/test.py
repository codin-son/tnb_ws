import pyrealsense2 as rs
import numpy as np

# Initialize the RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Configure depth stream

# Start streaming
pipeline.start(config)

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()

        if not depth_frame:
            continue

        # Convert depth frame to numpy array
        depth_image = np.asanyarray(depth_frame.get_data())

        # Calculate the center of the image
        center_y, center_x = depth_image.shape[0] // 2, depth_image.shape[1] // 2

        # Get the depth value at the center of the image
        depth_value = depth_image[center_y, center_x]
        print("Depth value at center:", depth_value)

finally:
    # Stop streaming
    pipeline.stop()