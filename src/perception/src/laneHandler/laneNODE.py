#!/usr/bin/env python3

import cv2
import numpy as np

import socket
import struct
import time
import rospy
import lanesImg 

from cv_bridge       import CvBridge
from sensor_msgs.msg import Image


class laneNode():
    id = 0
    def __init__(self):
        rospy.init_node('laneNODE', anonymous=False)
        
        rospy.Subscriber("/automobile/image_raw", Image, self._streams)
        self.lane_publiser = rospy.Publisher("/automobile/lane_image", Image, queue_size=1)
        
        self.bridge = CvBridge()

    def run(self):
        rospy.loginfo("starting laneNODE")
        rospy.spin()
    
    def _streams(self, msg):
        self.id += 1
        stringId = "/home/pi/tests/ROS_test1" + str(self.id) + ".jpg"
        image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
     
        cv2.imwrite(stringId, image)       
        cv2.waitKey(1)

        #laneImg = lanesImg.imageTransform(image)
       
        cv2.imshow('Image', image)
        cv2.waitKey(1)
        
        #self.lane_publiser.publish(laneImg)
        
if __name__ == "__main__":
    laneNod = laneNode()
    laneNod.run()
