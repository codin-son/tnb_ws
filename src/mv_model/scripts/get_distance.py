#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import Float32  # Import the Float32 message type

class DepthSubscriber:
    def __init__(self):
        rospy.init_node('depth_subscriber', anonymous=True)
        self.bridge = CvBridge()
        self.depth_image_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)
        self.depth_pub = rospy.Publisher('depth_value', Float32, queue_size=10)  # Create a publisher for depth_value

    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth_array = np.array(depth_image, dtype=np.float32)
            height, width = depth_array.shape[:2]
            center_x = width // 2
            center_y = height // 2
            depth_value = depth_array[center_y, center_x]
            rospy.loginfo("Depth value at the center (x=%d, y=%d): %f mm", center_x, center_y, depth_value)
            self.depth_pub.publish(depth_value)
        except Exception as e:
            rospy.logerr(e)

if __name__ == '__main__':
    try:
        depth_subscriber = DepthSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass