#!/bin/python3
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class video_reader:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/camera/color/image_raw", Image, queue_size=10)
    def read(self):
        cap = cv2.VideoCapture("rec.mp4")
        while cap.isOpened():
            try:
                ret, frame = cap.read()
                cv2.imshow('demo',frame)
                if cv2.waitKey(1) == ord('q'):
                    break
                if ret == True:
                    img_message = self.bridge.cv2_to_imgmsg(frame)
                    self.image_pub.publish(img_message)
            except:
                pass

def main():
    rospy.init_node('video_reader', anonymous=True)
    vr = video_reader()
    vr.read()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()