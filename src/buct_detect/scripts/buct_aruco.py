#!/bin/python3
from operator import is_not
import rospy
import numpy as np
import cv2
import cv2.aruco as aruco
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from buct_msgs.msg import Target
from buct_msgs.msg import TargetList
import tf
import sys
from scipy.spatial.transform import Rotation as R
import math

aruco_dict = aruco.custom_dictionary(8,5,1)
aruco_dict.bytesList = np.empty(shape = (8, 4, 4), dtype = np.uint8)

mybits0 = np.array([[0,1,1,0,0],[1,0,1,0,0],[0,0,1,0,0],[0,0,1,0,0],[1,1,1,1,1]], dtype = np.uint8) # number '1'
mybits1 = np.array([[0,1,1,1,0],[1,0,0,0,1],[0,0,0,1,0],[0,0,1,0,0],[1,1,1,1,1]], dtype = np.uint8) # number '2'
mybits2 = np.array([[0,1,1,1,0],[1,0,0,0,1],[0,0,1,1,0],[1,0,0,0,1],[0,1,1,1,0]], dtype = np.uint8) # number '3'
mybits3 = np.array([[0,0,0,1,0],[0,0,1,1,0],[0,1,0,1,0],[1,1,1,1,1],[0,0,0,1,0]], dtype = np.uint8) # number '4'
mybits4 = np.array([[1,1,1,1,1],[1,0,0,0,0],[0,1,1,1,0],[0,0,0,0,1],[1,1,1,1,0]], dtype = np.uint8) # number '5'
mybits5 = np.array([[1,1,1,1,0],[1,0,0,0,1],[1,1,1,1,0],[1,0,0,0,1],[1,1,1,1,0]], dtype = np.uint8) # letter 'B'
mybits6 = np.array([[0,1,1,1,0],[1,0,0,0,1],[1,0,0,0,1],[1,0,0,0,1],[0,1,1,1,0]], dtype = np.uint8) # letter 'O'
mybits7 = np.array([[1,0,0,0,1],[0,1,0,1,0],[0,0,1,0,0],[0,1,0,1,0],[1,0,0,0,1]], dtype = np.uint8) # letter 'X'

aruco_dict.bytesList[0] = aruco.Dictionary_getByteListFromBits(mybits0) # number '1'
aruco_dict.bytesList[1] = aruco.Dictionary_getByteListFromBits(mybits1) # number '2'
aruco_dict.bytesList[2] = aruco.Dictionary_getByteListFromBits(mybits2) # number '3'
aruco_dict.bytesList[3] = aruco.Dictionary_getByteListFromBits(mybits3) # number '4'
aruco_dict.bytesList[4] = aruco.Dictionary_getByteListFromBits(mybits4) # number '5'
aruco_dict.bytesList[5] = aruco.Dictionary_getByteListFromBits(mybits5) # letter 'B'
aruco_dict.bytesList[6] = aruco.Dictionary_getByteListFromBits(mybits6) # letter 'O'
aruco_dict.bytesList[7] = aruco.Dictionary_getByteListFromBits(mybits7) # letter 'X'


# generate aruco pictures
'''
for i in range(len(aruco_dict.bytesList)):
    cv2.imwrite("./dict/custom_aruco_" + str(i) + ".png", aruco.drawMarker(aruco_dict, i, 128))
'''
    
class arucoPose:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.imageColorCallback)
        self.camera_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.cameraInfoCallback)
        self.image_pub = rospy.Publisher("/buct/detected_image", Image, queue_size=10)
        self.buct_pub = rospy.Publisher("/buct/target_list", TargetList, queue_size=10)
        self.MARKER_SIZE = 0.045
        self.ARUCO_DICT = aruco_dict
        self.MTX = np.array([[0,0,0],[0,0,0],[0,0,0]])
        self.DIST = np.array([0,0,0,0,0])
        self.already_get_camera_info = False

        self.id_list = []
        self.tvec_list = []
        self.rvec_list = []
        self.tf_listener = tf.TransformListener()
        try:
            self.tf_listener.waitForTransform('/map', '/grasp_position_first', rospy.Time(), rospy.Duration(2.0))
            self.tf_listener.waitForTransform('/map', '/grasp_position_second', rospy.Time(), rospy.Duration(2.0))
            self.tf_listener.waitForTransform('/map', '/grasp_position_third', rospy.Time(), rospy.Duration(2.0))
            self.tf_listener.waitForTransform('/map', '/grasp_position_fourth', rospy.Time(), rospy.Duration(2.0))
            self.tf_listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(1.0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            return
 
    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
        
    def get_position(self):
        try:
            (trans1, rot1) = self.tf_listener.lookupTransform('/map','/grasp_position_first', rospy.Time(0))
            (trans2, rot2) = self.tf_listener.lookupTransform('/map','/grasp_position_second', rospy.Time(0))
            (trans3, rot3) = self.tf_listener.lookupTransform('/map','/grasp_position_third', rospy.Time(0))
            (trans4, rot4) = self.tf_listener.lookupTransform('/map','/grasp_position_fourth', rospy.Time(0))
            (trans5, rot5) = self.tf_listener.lookupTransform('/map','/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #rospy.loginfo("tf Error")
            return None 
        return trans1,trans2,trans3,trans4,trans5,rot1,rot2,rot3,rot4,rot5

    def select_position(self,id):
        t_final = []
        r_final = []
        try:
            t1,t2,t3,t4,t5,r1,r2,r3,r4,r5 = self.get_position()
            t_list = [t1,t2,t3,t4]
            r_list = [r1,r2,r3,r4]
            t_list_satisfy_xy_req = []
            r_list_satisfy_xy_req = []
            if(id == '1'):
                for i in range(len(t_list)):
                    t = t_list[i]
                    r = r_list[i]
                    roll,pitch,yaw = self.euler_from_quaternion(r[0], r[1], r[2], r[3])
                    yaw = yaw*180/math.pi
                    if(yaw>=90 or yaw<=-175):
                        t_list_satisfy_xy_req.append(t)
                        r_list_satisfy_xy_req.append(r)
                dist_min = 10.0
                for i in range(len(t_list_satisfy_xy_req)):
                    t = t_list_satisfy_xy_req[i]
                    r = r_list_satisfy_xy_req[i]
                    dist = abs(t5[0]-t[0]) + abs(t5[1]-t[1])
                    if(dist <= dist_min):
                        dist_min = dist
                        t_final = t
                        r_final = r
            elif(id == '2'):
                for i in range(len(t_list)):
                    t = t_list[i]
                    r = r_list[i]
                    roll,pitch,yaw = self.euler_from_quaternion(r[0], r[1], r[2], r[3])
                    yaw = yaw*180/math.pi
                    if(yaw<=0 and yaw>=-90):
                        t_list_satisfy_xy_req.append(t)
                        r_list_satisfy_xy_req.append(r)
                dist_min = 10.0
                for i in range(len(t_list_satisfy_xy_req)):
                    t = t_list_satisfy_xy_req[i]
                    r = r_list_satisfy_xy_req[i]
                    dist = abs(t5[0]-t[0]) + abs(t5[1]-t[1])
                    if(dist <= dist_min):
                        dist_min = dist
                        t_final = t
                        r_final = r
            elif(id == '3'):
                for i in range(len(t_list)):
                    t = t_list[i]
                    r = r_list[i]
                    roll,pitch,yaw = self.euler_from_quaternion(r[0], r[1], r[2], r[3])
                    yaw = yaw*180/math.pi
                    if(((yaw>=0 and yaw<=7) or (yaw<=89 and yaw>=83)) and t[0]<=2.8):
                        t_list_satisfy_xy_req.append(t)
                        r_list_satisfy_xy_req.append(r)
                dist_min = 10.0
                for i in range(len(t_list_satisfy_xy_req)):
                    t = t_list_satisfy_xy_req[i]
                    r = r_list_satisfy_xy_req[i]
                    dist = abs(t5[0]-t[0]) + abs(t5[1]-t[1])
                    if(dist <= dist_min):
                        dist_min = dist
                        t_final = t
                        r_final = r
            elif(id == '4'):
                for i in range(len(t_list)):
                    t = t_list[i]
                    r = r_list[i]
                    roll,pitch,yaw = self.euler_from_quaternion(r[0], r[1], r[2], r[3])
                    yaw = yaw*180/math.pi
                    if(yaw<=-110 or yaw>=110):
                        t_list_satisfy_xy_req.append(t)
                        r_list_satisfy_xy_req.append(r)
                dist_min = 10.0
                for i in range(len(t_list_satisfy_xy_req)):
                    t = t_list_satisfy_xy_req[i]
                    r = r_list_satisfy_xy_req[i]
                    dist = abs(t5[0]-t[0]) + abs(t5[1]-t[1])
                    if(dist <= dist_min):
                        dist_min = dist
                        t_final = t
                        r_final = r
            elif(id == '5'):
                for i in range(len(t_list)):
                    t = t_list[i]
                    r = r_list[i]
                    roll,pitch,yaw = self.euler_from_quaternion(r[0], r[1], r[2], r[3])
                    yaw = yaw*180/math.pi
                    if(yaw<=5 and yaw>=-5):
                        t_list_satisfy_xy_req.append(t)
                        r_list_satisfy_xy_req.append(r)
                dist_min = 10.0
                for i in range(len(t_list_satisfy_xy_req)):
                    t = t_list_satisfy_xy_req[i]
                    r = r_list_satisfy_xy_req[i]
                    dist = abs(t5[0]-t[0]) + abs(t5[1]-t[1])
                    if(dist <= dist_min):
                        dist_min = dist
                        t_final = t
                        r_final = r
        except:
            pass
        #rospy.loginfo(str(t_final)+' '+str(r_final))
        if(len(t_final)==3 and len(r_final)==4):
            return t_final,r_final

    def cameraInfoCallback(self,data):
        if not(self.already_get_camera_info):
            k = data.K
            d = data.D
            self.MTX = np.array([[k[0], k[1], k[2]], 
                                [k[3], k[4], k[5]], 
                                [k[6], k[7], k[8]]])
            self.DIST = np.array([d[0], d[1], d[2], d[3], d[4]])
            self.already_get_camera_info = True
    def imageColorCallback(self,data):
        try:
            img_rgb = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as err:
            print(err)

        # convert red into black
        float_img_rgb = np.array(img_rgb,dtype=np.float32) / 255.0
        (b, g, r) = cv2.split(float_img_rgb)
        gray = 255 - 2*r + g + b
        (minval, maxval, minloc, maxloc) = cv2.minMaxLoc(gray)
        gray_u8 = np.array((gray-minval)/(maxval-minval)*255,dtype=np.uint8)
        (thresh, bin_img) = cv2.threshold(gray_u8,-1.0,255,cv2.THRESH_OTSU)
        frame = bin_img

        # detect markers
        aruco_parameters = cv2.aruco.DetectorParameters_create()
        corners, marker_ids, rejectedImgPoints = aruco.detectMarkers(
            frame, self.ARUCO_DICT,parameters=aruco_parameters,
            cameraMatrix=self.MTX, distCoeff=self.DIST)
        frame = aruco.drawDetectedMarkers(img_rgb, corners, marker_ids)
        if marker_ids is not None:
            rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
                corners,
                self.MARKER_SIZE,
                self.MTX,
                self.DIST)
            target_list_msg = []
            for i, marker_id in enumerate(marker_ids):
                aruco_pose_msg = Pose()

                aruco_pose_msg.position.x = tvecs[i][0][0]
                aruco_pose_msg.position.y = tvecs[i][0][1]
                aruco_pose_msg.position.z = tvecs[i][0][2]

                rotation_matrix = np.eye(4)
                rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                r = R.from_matrix(rotation_matrix[0:3, 0:3])
                quat = r.as_quat()   

                aruco_pose_msg.orientation.x = quat[0] 
                aruco_pose_msg.orientation.y = quat[1] 
                aruco_pose_msg.orientation.z = quat[2] 
                aruco_pose_msg.orientation.w = quat[3]

                target_msg = Target()
                if(marker_id[0] == 0):
                    target_msg.id = '1'
                elif(marker_id[0] == 1):
                    target_msg.id = '2'
                elif(marker_id[0] == 2):
                    target_msg.id = '3'
                elif(marker_id[0] == 3):
                    target_msg.id = '4'
                elif(marker_id[0] == 4):
                    target_msg.id = '5'
                elif(marker_id[0] == 5):
                    target_msg.id = 'B'
                elif(marker_id[0] == 6):
                    target_msg.id = 'O'
                elif(marker_id[0] == 7):
                    target_msg.id = 'X'                                                                                                              
                target_msg.pose = aruco_pose_msg
                target_list_msg.append(target_msg)
                
                px = aruco_pose_msg.position.x
                py = aruco_pose_msg.position.y
                pz = aruco_pose_msg.position.z
                qx = aruco_pose_msg.orientation.x
                qy = aruco_pose_msg.orientation.y
                qz = aruco_pose_msg.orientation.z
                qw = aruco_pose_msg.orientation.w
                obj_tf = tf.TransformBroadcaster()
                obj_tf.sendTransform((px,py,pz),(qx,qy,qz,qw),rospy.Time.now(),target_msg.id+', '+str(i),'camera_color_optical_frame')
                if(abs(py)<=0.0225):
                    if(target_msg.id == '1' or target_msg.id == '2' or target_msg.id == '4'):
                        obj_tf.sendTransform((0.0325+0.00,-0.025,0.4256),(-0.5, 0.5, 0.5, 0.5),rospy.Time.now(),'grasp_position_first',target_msg.id+', '+str(i))
                        obj_tf.sendTransform((0.3581,0.3581,0),(0,0,-0.707,0.707),rospy.Time.now(),'grasp_position_second','grasp_position_first')
                        obj_tf.sendTransform((0.3581,0.3581,0),(0,0,-0.707,0.707),rospy.Time.now(),'grasp_position_third','grasp_position_second')
                        obj_tf.sendTransform((0.3581,0.3581,0),(0,0,-0.707,0.707),rospy.Time.now(),'grasp_position_fourth','grasp_position_third')
                    elif(target_msg.id == '3' or target_msg.id == '5'):
                        obj_tf.sendTransform((0.0325+0.05,-0.025,0.5556),(-0.5, 0.5, 0.5, 0.5),rospy.Time.now(),'grasp_position_first',target_msg.id+', '+str(i))
                        obj_tf.sendTransform((0.4881,0.4881,0),(0,0,-0.707,0.707),rospy.Time.now(),'grasp_position_second','grasp_position_first')
                        obj_tf.sendTransform((0.4881,0.4881,0),(0,0,-0.707,0.707),rospy.Time.now(),'grasp_position_third','grasp_position_second')
                        obj_tf.sendTransform((0.4881,0.4881,0),(0,0,-0.707,0.707),rospy.Time.now(),'grasp_position_fourth','grasp_position_third')

                    try:
                        t,r = self.select_position(str(target_msg.id))
                        if(len(t)==3 and len(r)==4):
                            obj_tf.sendTransform((t[0],t[1],t[2]),(r[0],r[1],r[2],r[3]),rospy.Time.now(),'grasp_position'+target_msg.id,'map')
                    except:
                        rospy.loginfo('get position failed')
                cv2.aruco.drawAxis(frame, self.MTX, self.DIST, rvecs[i], tvecs[i], 0.02)
            self.buct_pub.publish(target_list_msg)

        try:
            self.image_message = self.bridge.cv2_to_imgmsg(frame, "passthrough")
        except CvBridgeError as e:
            print(e)
        self.image_pub.publish(self.image_message)

def main():
    rospy.init_node('buct_aruco_node', anonymous=True)
    ap = arucoPose()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
