#!/usr/bin/env python3
import rospy
import cv2
from torch import IntTensor
import numpy as np
from ultralytics import YOLO
import time


model = YOLO("/home/a2tech/tnb_ws/src/mv_model/scripts/runs_updated/content/runs/detect/train/weights/best.pt")

def vshDetector(pic):
    
    result = model.predict(source=pic, conf=0.7, verbose=False)
    
    if result[0]:
        start_conf = 0
        current_level_conf = 0
        for box in result[0].boxes:
            cls = IntTensor.item(box.cls)
            conf = IntTensor.item(box.conf)

            if cls == 1:
                if conf > start_conf:
                    start_conf = conf
                    start_box = box
            else:
                if conf > current_level_conf:
                    current_level_conf = conf
                    current_level_box = box
        try:
            start_line = int(start_box.xyxy[0][3].item() * 480 / 720) - 5
            current_level_line = int(current_level_box.xyxy[0][3].item() * 480 / 720)
            depth_point = [start_line, int(start_box.xyxy[0][2].item() * 640 / 1280)]
            margin_px = abs(current_level_line - start_line)
        except:            
            return 0

        return [start_line, current_level_line, margin_px, depth_point]
        

def TravelMargin(lines, pic, margin_mm):
    # pic = cv2.resize(pic, (320, 240))
    pic = pic.copy()
    if lines:
        h, w, c = pic.shape
        # rospy.loginfo(h)
        # rospy.loginfo(w)

        start_line = int(lines[0] * 720 / 480)
        current_level_line = int(lines[1] * 720 / 480)
        # get difference between start line and current level line for y axis
        margin_px = abs(current_level_line - start_line)


        pic = cv2.line(pic, (0, start_line), (w, start_line), (0, 0, 255), 1)
        pic = cv2.line(pic, (0, current_level_line), (w, current_level_line), (0, 255, 0), 1)
        pic = cv2.line(pic, (int(w*0.8), start_line), (int(w*0.8), current_level_line), (255, 0, 0), 3)
        pic = cv2.rectangle(pic, (int(w*0.85), int(h/2)), (int(w*0.99), int(h*0.55)), (255, 0, 0), -1)
        pic = cv2.putText(pic, str(margin_mm) + "mm", (int(w*0.85), int(h*0.55)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

        return pic
    else:

        return pic 

    # print(margin_px)
            
    # cv2.imwrite("op.png", pic) 

# pic = "/home/mohammed/Documents/VSH_model/dataset/9.jpg"
# TravelMargin(vshDetector(pic), pic)