#!/usr/bin/python

import cv_bridge
import cv2
import sys
import rospy
import numpy as np
import baxter_interface
import rospkg
import os
from baxter_interface import CHECK_VERSION, CameraController


from sensor_msgs.msg import(
    Image
)

rospack = rospkg.RosPack()
cascPath = os.path.join(rospack.get_path("lab_common"), 'opencv-3.4.1/data/haarcascades/haarcascade_frontalface_default.xml')
faceCascade = cv2.CascadeClassifier(cascPath)

video_capture = cv2.VideoCapture(0)

class CameraNode:
    def __init__(self):
        self._head_sub = rospy.Subscriber('/cameras/head_camera/image', Image, self._head_cb, queue_size=1)
        self._last_image = None
        self._head = CameraController('head_camera')
        self._head.open()
        self._head.resolution = (1280, 800)
    def _head_cb(self, msg):
        self._last_image = msg
    def detect(self):  
        while True:
            # Capture frame-by-frame
            if self._last_image != None:
                frame = cv_bridge.CvBridge().imgmsg_to_cv2(self._last_image, desired_encoding='bgr8')
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                faces = faceCascade.detectMultiScale(
                    gray,
                    scaleFactor=1.1,
                    minNeighbors=5,
                    minSize=(30, 30),
                    flags=cv2.CASCADE_SCALE_IMAGE
                )

                # Draw a rectangle around the faces
                for (x, y, w, h) in faces:
                    head = baxter_interface.Head()
                    if x < 400:
                        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                        head.set_pan(head.pan() + 0.35) 
                    elif x < 550:
                        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                        head.set_pan(head.pan() + 0.15)
                    if x > 800:
                        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                        head.set_pan(head.pan() - 0.35)
                    elif x > 650:
                        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                        head.set_pan(head.pan() - 0.15)
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    cv2.rectangle(frame, (600, 380), (640, 420), (255, 0, 0), 2)
                    
                # Display the resulting frame
                cv2.imshow('Video', frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        # When everything is done, release the capture
        video_capture.release()
        cv2.destroyAllWindows()

