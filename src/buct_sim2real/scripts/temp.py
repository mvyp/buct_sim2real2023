#!/usr/bin/env python3
import rospy

from tf_conversions import transformations
from math import pi
import tf
import numpy as np
from geometry_msgs.msg import Twist
class Robot:
    def __init__(self):
        self.detect_state = False
        self.tf_listener = tf.TransformListener()
        self.publisher = rospy.Publisher('/cmd_position', Twist)
        try:
            self.tf_listener.waitForTransform('/camera_color_optical_frame', '/cube', rospy.Time(), rospy.Duration(1.0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            return

    def get_pos(self):
        # try:
            self.detect_state = True
            (trans, rot) = self.tf_listener.lookupTransform('/camera_color_optical_frame', '/cube', rospy.Time(0))
            print(trans)
            mat = transformations.quaternion_matrix(rot)
            axis_y = mat[1][:3]
            axis_retain = [[] for _ in range(3)]
            for i,value in enumerate(axis_y):
                if value < 0.03:
                    for j in range(3):
                        axis_retain[j].append(mat[j][i])
            print(axis_retain)
            best_axis_index = axis_retain[0].index(min(axis_retain[0]))
            x = axis_retain[0][best_axis_index]
            z = axis_retain[2][best_axis_index]
            direction = np.sign(x) #turn left : 1 ; trun right: -1
            tan = abs(x/z)
            axis2angle = np.arctan(tan)
            t = trans[0] + trans[2] * np.tan(axis2angle * direction)
            #print euler[2] / pi * 180
            print(axis2angle,t)
            return axis2angle,direction,-t
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException,ValueError):
            # self.detect_state = False
            # rospy.loginfo("tf Error")
            # return None
    def pub_position(self):
        self.get_pos()
        if self.detect_state:
            while True:
                angle,direction,trans = self.get_pos()
                pub_data = Twist()
                pub_data.linear.y =  trans 
                self.publisher.publish(pub_data)
                rospy.sleep(abs(trans)/0.1)
                pub_data = Twist()
                pub_data.angular.z = angle * direction
                self.publisher.publish(pub_data)
                rospy.sleep(2)
                (trans, rot) = self.tf_listener.lookupTransform('/camera_color_optical_frame', '/cube', rospy.Time(0))
                mat = transformations.quaternion_matrix(rot)
                if abs(trans[0]) < 0.1:
                    break
        

        # x = trans[0]
        # y = trans[1]
        # th = euler[2] / pi * 180
        # return (x, y, th)

if __name__ == "__main__":
    rospy.init_node('get_pos_demo',anonymous=True)
    # robot = Robot()
    # r = rospy.Rate(10)
    # r.sleep()
    # while not rospy.is_shutdown():
    #     try:
    #         angle,direction,trans = robot.get_pos()
    #         pub_data = Twist()
    #         pub_data.linear.y =  trans
    #         pub_data.angular.z = angle * direction
    #         robot.publisher.publish(pub_data)
    #     except TypeError:
    #         print('no detected object')
    #     r.sleep()
    robot = Robot()
    robot.pub_position()
    # while True:
    #     angle,direction,trans = robot.get_pos()
    #     pub_data = Twist()
    #     pub_data.linear.y =  trans 
    #     robot.publisher.publish(pub_data)
    #     rospy.sleep(abs(trans)/0.1)
    #     pub_data = Twist()
    #     pub_data.angular.z = angle * direction
    #     robot.publisher.publish(pub_data)
    #     rospy.sleep(2)
    #     (trans, rot) = robot.tf_listener.lookupTransform('/camera_color_optical_frame', '/cube', rospy.Time(0))
    #     mat = transformations.quaternion_matrix(rot)
    #     if abs(trans[0]) < 0.1:
    #         break



    