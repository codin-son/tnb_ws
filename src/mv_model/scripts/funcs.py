#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
def topic_to_img(data):
    try:
        np_arr = np.fromstring(data.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        img = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    except Exception as e:
        rospy.loginfo(e)

    return img

def img_to_topic(img):
    bridge = CvBridge()
    try:
        data = bridge.cv2_to_imgmsg(img, encoding='rgb8')
    except CvBridgeError as e:
        rospy.loginfo(e)

    return data