#!/bin/python3
from operator import is_not
import rospy
import numpy as np
import cv2
# from utils import ARUCO_DICT
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from buct_msgs.msg import Target
from buct_msgs.msg import TargetList
import tf
import sys

from redmarkerdetection import *    # image processing by cython

def pose_aruco_2_ros(rvec, tvec):
    aruco_pose_msg = Pose()
    aruco_pose_msg.position.x = tvec[0]
    aruco_pose_msg.position.y = tvec[1]
    aruco_pose_msg.position.z = tvec[2]
    rotation_matrix = np.array([[0, 0, 0, 0],
                                [0, 0, 0, 0],
                                [0, 0, 0, 0],
                                [0, 0, 0, 1]],
                                dtype=float)
    rotation_matrix[:3, :3], _ = cv2.Rodrigues(rvec)
    quaternion = tf.transformations.quaternion_from_matrix(rotation_matrix)
    aruco_pose_msg.orientation.x = quaternion[0]
    aruco_pose_msg.orientation.y = quaternion[1]
    aruco_pose_msg.orientation.z = quaternion[2]
    aruco_pose_msg.orientation.w = quaternion[3]
    return aruco_pose_msg
    
class arucoPose:
    def __init__(self):
        self.buct_pub = rospy.Publisher("/buct/target_list", TargetList)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.imageColorCallback)
        self.image_pub = rospy.Publisher("/buct/detected_image", Image)

        self.MARKER_SIZE = 0.045 # [m]
        self.ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.MTX = np.array([[617.3054000792732, 0.0, 424.0], 
                        [0.0, 617.3054000792732, 240.0], 
                        [0.0, 0.0, 1.0]])
        self.DIST = np.array([0., 0., 0., 0., 0.])

        self.id_list = []
        self.tvec_list = []
        self.rvec_list = []

        load_template()

    def imageColorCallback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as err:
            print(err)
            
        (h, w, channel) = cv_image.shape
        seg_papram = np.array([0,15,125,180,46,80],dtype="uint8")
        id_list,tvec_list,rvec_list = marker_detection(cv_image,seg_papram)
        
        
        try:
            self.image_message = self.bridge.cv2_to_imgmsg(cv_image, "passthrough")
        except CvBridgeError as e:
            print(e)
        self.image_pub.publish(self.image_message)

        # publish pose to rostopic
        target1_detected = False
        target2_detected = False
        target3_detected = False
        target4_detected = False
        target5_detected = False
        sink1_detected = False
        sink2_detected = False
        sink3_detected = False
        
        target_list_msg = []
        
        for i in range(len(id_list)):
            aruco_pose_msg = pose_aruco_2_ros(rvec_list[i],tvec_list[i])
            target_msg = Target()
            # 1
            if id_list[i] == 0 and target1_detected == False:
                target_msg.id = '1'
                target_msg.pose = aruco_pose_msg
                target_list_msg.append(target_msg)
                target1_detected = True
            # 2
            if id_list[i] == 1 and target2_detected == False:
                target_msg.id = '2'
                target_msg.pose = aruco_pose_msg
                target_list_msg.append(target_msg)
                target2_detected = True
            # 3     
            if id_list[i] == 2 and target3_detected == False:
                target_msg.id = '3'
                target_msg.pose = aruco_pose_msg
                target_list_msg.append(target_msg)
                target3_detected = True
            # 4     
            if id_list[i] == 6 and target4_detected == False:
                target_msg.id = '4'
                target_msg.pose = aruco_pose_msg
                target_list_msg.append(target_msg)
                target4_detected = True
            # 5     
            if id_list[i] == 7 and target5_detected == False:
                target_msg.id = '5'
                target_msg.pose = aruco_pose_msg
                target_list_msg.append(target_msg)
                target5_detected = True
                    
            # B
            if id_list[i] == 3 and sink1_detected == False:
                target_msg.id = 'B'
                target_msg.pose = aruco_pose_msg
                target_list_msg.append(target_msg)
                skin1_detected = True
            # O
            if id_list[i] == 4 and sink2_detected == False:
                target_msg.id = 'O'
                target_msg.pose = aruco_pose_msg
                target_list_msg.append(target_msg)
                skin2_detected = True
            # X
            if id_list[i] == 5 and sink3_detected == False:
                target_msg.id = 'X'
                target_msg.pose = aruco_pose_msg
                target_list_msg.append(target_msg)
                skin3_detected = True
            px = aruco_pose_msg.position.x
            py = aruco_pose_msg.position.y
            pz = aruco_pose_msg.position.z
            qx = aruco_pose_msg.orientation.x
            qy = aruco_pose_msg.orientation.y
            qz = aruco_pose_msg.orientation.z
            qw = aruco_pose_msg.orientation.w
            obj_tf = tf.TransformBroadcaster()
            obj_tf.sendTransform((px,py,pz),(qx,qy,qz,qw),rospy.Time.now(),target_msg.id+str(i),'camera_color_optical_frame')
        self.buct_pub.publish(target_list_msg)


def main():
    rospy.init_node('aruco_pose_node', anonymous=True)
    ap = arucoPose()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
