#!/usr/bin/env python3
import rospy
from std_msgs.msg import String 
from buct_msgs.msg import *
import tf
from tf_conversions import transformations
import math
from math import pi
import numpy as np
from scipy.spatial.transform import Rotation as R
import random

class buct_sim2real:
    def __init__(self):
        self.mission_target_list = []
        self.mission_target_id_list = []

        self.target_list = TargetList()
        self.target_sub = rospy.Subscriber("/buct/target_list", TargetList, self.target_cb)
        self.detection_pub = rospy.Publisher("/buct_detect_result", String, queue_size=10)

    def target_cb(self,data):
        self.target_list = data

    def get_mission(self):
        # read
        self.mission_target_list = []
        self.mission_target_id_list = []
        try:
            for target in self.target_list.target_list:
                if (len(self.mission_target_list)!=3 and (target.id not in self.mission_target_id_list) and (target.id!='B') and (target.id!='O') and (target.id!='X')):
                    self.mission_target_list.append(target)
                    self.mission_target_id_list.append(target.id)
            # sort
            for j in range(len(self.mission_target_list)-1): 
                for i in range(len(self.mission_target_list)-1):
                    if(self.mission_target_list[i].pose.position.x > self.mission_target_list[i+1].pose.position.x):
                        temp_target = Target()
                        temp_target = self.mission_target_list[i]
                        temp_target_id = self.mission_target_id_list[i]
                        self.mission_target_list[i] = self.mission_target_list[i+1]
                        self.mission_target_id_list[i] = self.mission_target_id_list[i+1]
                        self.mission_target_list[i+1] = temp_target
                        self.mission_target_id_list[i+1] = temp_target_id
        except :
            pass
        
if __name__ == "__main__":
    rospy.init_node('buct_sim2real',anonymous=True)
    r = rospy.Rate(100)
    r.sleep()
    tantianwei = buct_sim2real()
    while(1):
        tantianwei.get_mission()
        if(len(tantianwei.mission_target_id_list)==3):
            tantianwei.detection_pub.publish(str(tantianwei.mission_target_id_list[0])+str(tantianwei.mission_target_id_list[1])+str(tantianwei.mission_target_id_list[2]))
        r.sleep()
