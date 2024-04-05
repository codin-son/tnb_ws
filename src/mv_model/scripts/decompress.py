#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import numpy as np

class DepthSubscriber:
    def __init__(self):
        rospy.init_node('depth_subscriber', anonymous=True)
        self.bridge = CvBridge()
        self.depth_image_sub = rospy.Subscriber('/camera/depth/image_rect_raw/compressedDepth', CompressedImage, self.depth_callback)

    def depth_callback(self, msg):
        try:
            with open('depth_message.txt', 'w') as file:
                file.write(str(msg))
            self.depth_image_sub.unregister()
        except Exception as e:
            rospy.logerr(e)

if __name__ == '__main__':
    try:
        depth_subscriber = DepthSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass