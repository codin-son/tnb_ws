#!/usr/bin/env python3

from funcs import topic_to_img, img_to_topic
from sensor_msgs.msg import Image, CompressedImage
from vsh_model import vshDetector, TravelMargin
from ocr_ops import detect_cert
from std_msgs.msg import Float32, Bool, String
from qr_reader import read_qr
import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError


class ImageProcessor:
    # Define constants
    COUNTER_RESET_VALUE = 5
    DEFAULT_Y = 240
    DEFAULT_X = 320
    
    # 0.002457444

    def __init__(self):
        self.lines = 0
        self.px2mm_ratio = 0
        self.cntr = self.COUNTER_RESET_VALUE
        self.travel_margin = 0
        self.margin_mm = 0
        self.img_pub = rospy.Publisher('travel_margin', Image, queue_size=1)
        self.scale_pub = rospy.Publisher('/get_scale_value', Float32, queue_size=1)
        self.ident_no = rospy.Publisher('/get_ident_id', String, queue_size=1)
        self.lock_qr_sub = rospy.Subscriber('/lock_qr_sub', Bool, self.lockqrCallback)
        self.is_lock_qr_pub = rospy.Publisher('/is_lock_qr', Bool, queue_size=1)
        self.swap_camSub = rospy.Subscriber('/swap_cam', Bool, self.swapCamCallback)
        self.recalibrateSub = rospy.Subscriber('/recalibrate', Bool, self.recalibrateCallback)
        self.realScaleSub = rospy.Subscriber('/real_scale', Float32, self.realScaleCallback)
        self.start_lock_qr = False
        self.swap_cam = True
        self.ident_id = None
        # self.PX2MM_RATIO_CONSTANT = 0.002457444 #saed
        # 0.0017661603673613563
        # self.PX2MM_RATIO_CONSTANT = 0.001546551190844417  #din1
        self.PX2MM_RATIO_CONSTANT = 0.001683690649904591  #din2
        self.margin_px = 0
        self.real_scale = None
        self.depth = None
        self.isRecalibrate = False
        self.qr_data = "N/A"
        self.bridge = CvBridge()

    def realScaleCallback(self, data):
        self.real_scale = data.data

    def recalibrateCallback(self, data):
        self.isRecalibrate = data.data
        if self.isRecalibrate:
            self.PX2MM_RATIO_CONSTANT = self.real_scale/(self.margin_px*self.depth)
            print(self.PX2MM_RATIO_CONSTANT)
            self.isRecalibrate = False

    def swapCamCallback(self, data):
        if data.data:
            self.swap_cam = True
        else:
            self.swap_cam = False


    def lockqrCallback(self, data):
        if data.data:
            self.start_lock_qr = True
        else:
            self.start_lock_qr = False

    def scan_scale(self,img):

        if self.cntr == self.COUNTER_RESET_VALUE:
            self.lines = vshDetector(img)

            try:
                self.margin_px = self.lines[2]
            except:
                self.margin_px = 0
            else:
                self.margin_mm = round(self.margin_px*self.px2mm_ratio, 1)
                
                self.cntr = 0

        else:
            self.cntr += 1

        output_img = TravelMargin(self.lines, img, self.margin_mm)
        return output_img


    def ocr_operation(self, input_img):
        h, w, c = input_img.shape
        if( h < 200 or w < 100):
            input_img = input_img
        else:
            a = cv2.imencode('.JPG', input_img)[1].tobytes()
            # Convert the byte data to a numpy array
            nparr = np.frombuffer(a, np.uint8)
            # Decode the numpy array as an image
            input_img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            gray_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2GRAY)
            gray_img_bytes = cv2.imencode('.JPG', gray_img)[1].tobytes()
            text_result = detect_cert(gray_img_bytes)

            return text_result
        
    def draw_crosshair_center(self, img, size=10, border_size=1):
        h, w, c = img.shape
        center = (int(w/2), int(h/2))

        # Draw larger crosshair for border
        img = cv2.line(img, (center[0], center[1] - size - border_size), (center[0], center[1] + size + border_size), (255, 255, 255), 3 + 2 * border_size)
        img = cv2.line(img, (center[0] - size - border_size, center[1]), (center[0] + size + border_size, center[1]), (255, 255, 255), 3 + 2 * border_size)

        # Draw original crosshair
        img = cv2.line(img, (center[0], center[1] - size), (center[0], center[1] + size), (255, 0, 0), 3)
        img = cv2.line(img, (center[0] - size, center[1]), (center[0] + size, center[1]), (255, 0, 0), 3)

        # Add white box
        cv2.rectangle(img, (10, 10), (200, 40), (255, 255, 255), -1)

        # Add text under the crosshair
        cv2.putText(img, f"Depth: {self.depth/10:.2f}cm", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

        return img
    def imageCallback(self, data):
        img = topic_to_img(data)
        # img = cv2.resize(img, (640, 480))
        if(self.swap_cam):
            img = self.scan_scale(img)
            self.scale_pub.publish(self.margin_mm)
        # img = cv2.resize(img, (1280, 720))
            
        img = self.draw_crosshair_center(img)
        

        
        if self.start_lock_qr == False:
            img, self.qr_data = read_qr(img)
        
        self.ident_no.publish(self.qr_data)
        


        img = img_to_topic(img)
        self.img_pub.publish(img)
        self.is_lock_qr_pub.publish(self.start_lock_qr)
        

    def depthCallback(self, data):

        self.depth = data.data
        
        self.px2mm_ratio = self.PX2MM_RATIO_CONSTANT * self.depth

if __name__ == '__main__':
    rospy.init_node('vsh_detect', anonymous=True)
    rospy.loginfo('vsh_detect is on')

    processor = ImageProcessor()

    # rospy.Subscriber("/camera/depth/image_rect_raw/compressed", CompressedImage, processor.depthCallback)
    rospy.Subscriber("/depth_value", Float32, processor.depthCallback)
    rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage, processor.imageCallback)

    rospy.spin()















    # roslaunch realsense2_camera rs_camera.launch color_width:=1280 color_height:=720 color_fps:=10 depth_width:=1280 depth_height:=720 depth_fps:=10 initial_reset:=true