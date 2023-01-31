#!/usr/bin/env python3
import rospy
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from buct_msgs.msg import *
import tf
from tf_conversions import transformations
import math
from math import pi
import numpy as np
from scipy.spatial.transform import Rotation as R
from tf_conversions import transformations
import random

class buct_sim2real:
    def __init__(self):
        self.state = 0
        self.gama_x = 0.015
        self.gama_y = 0.015
        self.gama_z = rospy.get_param('~/move_base/TebLocalPlannerROS/yaw_goal_tolerance')
        self.min_vel = 0.25
        self.goal = []
        self.goal_grasp = [0.045, 0.0, 0.25]
        self.goal_place = [0.045, 0.0, 0.23]
        self.move_base_goal = []
        self.going_to_move_base_goal = 0
        self.target_id = ' '
        self.trans_cache = []
        self.rot_cache = []
        self.mission_target_list = []
        self.mission_target_id_list = []

        self.move_vel_pub = rospy.Publisher("cmd_vel", Twist)
        self.move_position_pub = rospy.Publisher("cmd_position", Twist)
        self.arm_gripper_pub = rospy.Publisher("arm_gripper", Point)
        self.arm_position_pub = rospy.Publisher("arm_position", Pose)
        
        self.target_list = TargetList()
        self.target_sub = rospy.Subscriber("/buct/target_list", TargetList, self.target_cb)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))

        self.tf_listener = tf.TransformListener()
        try:
            self.tf_listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(2.0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            return

    def euler_from_quaternion(self, x, y, z, w):
        roll_x = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        yaw_z = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        return roll_x, pitch_y, yaw_z # in radians

    def transform_matrix_from_quaternion_and_position(self,ox,oy,oz,ow,x,y,z):
        matrix = np.array([[1-2*oy**2-2*oz**2,2*ox*oy-2*oz*ow,2*ox*oz+2*oy*ow,x],
                            [2*ox*oy+2*oz*ow,1-ox**2-oz**2,2*oy*oz-2*ox*ow,y],
                            [2*ox*oz-2*oy*ow,2*oy*oz+2*ox*ow,1-2*ox**2-2*oy**2,z],
                            [0,0,0,1]])
        return matrix

    def get_pos(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #rospy.loginfo("tf Error")
            return False
        euler = transformations.euler_from_quaternion(rot)
        x = trans[0]
        y = trans[1]
        th = euler[2] / pi * 180
        return (x, y, th)

    def target_cb(self,data):
        self.target_list = data
        if(self.state==0):
            counter = 0
            for target in self.target_list.target_list:
                if(target.id=='O'):
                    target.id='3'
                if ((target.pose.position.y<=-0.1)  and (target.id!='B') and (target.id!='X')):
                    counter += 1
            if(counter==3):
                self.state = 1
                self.move_base.cancel_goal()

    def done_cb(self, status, result):
        pass

    def active_cb(self):
        pass

    def feedback_cb(self, feedback):
        pass
        
    def goto_get_mission(self, x, y, th):
        self.move_base.cancel_goal()
        p = [x, y, th]
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p[0]
        goal.target_pose.pose.position.y = p[1]
        q = transformations.quaternion_from_euler(0.0, 0.0, p[2]/180.0*pi)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        self.move_base.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        result = self.move_base.wait_for_result(rospy.Duration(15))
        if not result:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("reach goal %s succeeded!"%p)
        return True

    def cancel_goal(self):
        for target in self.target_list.target_list:
            rospy.loginfo("detect id: "+str(target.id)+' target id: '+str(self.target_id))
            if ((target.id == self.target_id) and (target.pose.position.y>=-0.1)):
                return True
        return False  

    def goto(self, x, y, th):
        self.move_base.cancel_goal()
        self.going_to_move_base_goal = 1
        self.move_base_goal = [x,y,th]
        p = [x, y, th]
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p[0]
        goal.target_pose.pose.position.y = p[1]
        q = transformations.quaternion_from_euler(0.0, 0.0, p[2]/180.0*pi)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        self.move_base.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        while(1):
            (rx, ry, rz) = self.get_pos()
            x_error = abs(rx-x)
            y_error = abs(ry-y)
            yaw_error = abs(rz-th)*3.14/90
            if(self.target_id=='B' or self.target_id=='O' or self.target_id=='X'):
                if(x_error<=0.25 and y_error<=0.25 and yaw_error<=0.2):
                    if (self.cancel_goal()==True):
                        self.move_base.cancel_goal()
                        break
                    rospy.sleep(0.5)
            else:
                if(x_error<=0.25 and y_error<=0.25 and yaw_error<=0.5):
                    if (self.cancel_goal()==True):
                        self.move_base.cancel_goal()
                        break
                    rospy.sleep(0.5)
        result = self.move_base.wait_for_result(rospy.Duration(15))
        return True

    def get_mission(self):
        # read
        try:
            for target in self.target_list.target_list:
                if(target.id=='O'):
                    target.id='3'
                if (len(self.mission_target_list)!=3 and (target.id not in self.mission_target_id_list) and (target.pose.position.y<=-0.1)  and (target.id!='B') and (target.id!='X')):
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

    def open_gripper(self):
        open_gripper_msg = Point()
        open_gripper_msg.x = 0.0
        open_gripper_msg.y = 0.0
        open_gripper_msg.z = 0.0
        self.arm_gripper_pub.publish(open_gripper_msg)

    def close_gripper(self):
        close_gripper_msg = Point()
        close_gripper_msg.x = 1.0
        close_gripper_msg.y = 0.0
        close_gripper_msg.z = 0.0
        self.arm_gripper_pub.publish(close_gripper_msg)

    def move_arm(self):
        move_arm_msg = Pose()
        move_arm_msg.position.x = 0.2       # TODO
        move_arm_msg.position.y = -0.02
        move_arm_msg.position.z = 0
        move_arm_msg.orientation.x = 0.0
        move_arm_msg.orientation.y = 0.0
        move_arm_msg.orientation.z = 0.0
        move_arm_msg.orientation.w = 0.0
        self.arm_position_pub.publish(move_arm_msg)

    def move_arm0(self):
        move_arm_msg = Pose()
        move_arm_msg.position.x = 0.90      # TODO
        move_arm_msg.position.y = 0.12
        move_arm_msg.position.z = 0
        move_arm_msg.orientation.x = 0.0
        move_arm_msg.orientation.y = 0.0
        move_arm_msg.orientation.z = 0.0
        move_arm_msg.orientation.w = 0.0
        print("move the arm to the grasping pose")
        self.arm_position_pub.publish(move_arm_msg)

    def reset_arm(self):            
        reset_arm_msg = Pose()
        reset_arm_msg.position.x = 0.1
        reset_arm_msg.position.y = 0.12
        reset_arm_msg.position.z = 0.0
        reset_arm_msg.orientation.x = 0.0
        reset_arm_msg.orientation.y = 0.0
        reset_arm_msg.orientation.z = 0.0
        reset_arm_msg.orientation.w = 0.0
        self.arm_position_pub.publish(reset_arm_msg)
        rospy.sleep(0.1)
        self.arm_position_pub.publish(reset_arm_msg)

    def move_position(self, t_vector):
        move_base_msg_x = Twist()
        move_base_msg_x.linear.x = self.min_vel
        move_base_msg_x.angular.x = 0.0
        move_base_msg_x.angular.y = 0.0
        move_base_msg_x.angular.z = 0.0
        print("move the base in x direction")
        self.move_position_pub.publish(move_base_msg_x)
        
    def update_distance(self,mode):
        if(mode == 'grasp'):
            self.goal = self.goal_grasp
        elif(mode == 'place'):
            self.goal = self.goal_place
        target_z = 0
        target_x = 0
        # get sum
        target_list = []
        length = 0
        while(length==0):
            target_list = self.target_list.target_list
            for i in target_list:
                if(i.id == self.target_id and i.pose.position.y>=-0.1 and i.pose.position.y<=0.1):
                    tf1 = self.transform_matrix_from_quaternion_and_position(i.pose.orientation.x,i.pose.orientation.y,i.pose.orientation.z,i.pose.orientation.w,i.pose.position.x,i.pose.position.y,i.pose.position.z)
                    tf2 = self.transform_matrix_from_quaternion_and_position(0,0,0,1,0,0,-0.0225)
                    h = np.matmul(tf1,tf2)
                    # h is the homogeneous transform matrix of the inner centeral point of the box
                    # it corresponds to the transform from camera_optical_fram to the inner central point of the box
                    target_z += h[2][3] # position.z
                    target_x += h[0][3] # position.x
                    length += 1
                    obj_tf = tf.TransformBroadcaster()
                    obj_tf.sendTransform((h[0][3],h[1][3],h[2][3]),(0,0,-0.707,0.707),rospy.Time.now(),'abba','camera_color_optical_frame')
        # get mean value
        target_z /= length
        target_x /= length
        
        distance_in_x = target_z - self.goal[2]
        distance_in_y = target_x - self.goal[0]
        return distance_in_x, distance_in_y
    
    def adjust_pose(self,mode):
        cmd = Twist()
        cmd_stop = Twist()
        distance_in_x, distance_in_y = self.update_distance(mode)
        while(abs(distance_in_x) > self.gama_x or abs(distance_in_y) > self.gama_y):
            if (abs(distance_in_y) > self.gama_y and abs(distance_in_x) > self.gama_x):
                cmd.linear.y = -distance_in_y/abs(distance_in_y)*self.min_vel
                cmd.linear.x = distance_in_x/abs(distance_in_x)*self.min_vel
            elif(abs(distance_in_y) <= self.gama_y and abs(distance_in_x) > self.gama_x):
                cmd.linear.y = 0
                cmd.linear.x = distance_in_x/abs(distance_in_x)*self.min_vel
            elif(abs(distance_in_y) > self.gama_y and abs(distance_in_x) <= self.gama_x):
                cmd.linear.y = -distance_in_y/abs(distance_in_y)*self.min_vel
                cmd.linear.x = 0
            elif(abs(distance_in_y) <= self.gama_y and abs(distance_in_x) <= self.gama_x):
                cmd.linear.y = 0
                cmd.linear.x = 0
            self.move_vel_pub.publish(cmd)
            if(len(self.target_list.target_list)!=0):
                distance_in_x, distance_in_y = self.update_distance(mode)
            else:
                cmd_stop = Twist()
                self.move_vel_pub.publish(cmd_stop)
        self.move_vel_pub.publish(cmd_stop)
        
    def grasp_cube(self):
        self.reset_arm()
        self.open_gripper() 
        self.adjust_pose('grasp')
        position_cmd = Twist()
        position_cmd.linear.x = 0.1
        self.move_position_pub.publish(position_cmd)
        self.move_arm()
        rospy.sleep(0.9)
        self.close_gripper()
        rospy.sleep(0.1)
        self.reset_arm()
        position_cmd.linear.x = -0.09
        self.move_position_pub.publish(position_cmd)

    def place_cube(self):
        for i in self.target_list.target_list:
            if(i.id == self.target_id):
                target = i
        self.adjust_pose('place')
        self.reset_arm()
        self.move_arm0()
        position_cmd = Twist()
        position_cmd.linear.x = 0.18
        self.move_position_pub.publish(position_cmd)
        rospy.sleep(0.7)
        self.move_arm()
        rospy.sleep(0.9)
        self.open_gripper()
        rospy.sleep(0.1)
        position_cmd.linear.x = -0.18
        self.move_position_pub.publish(position_cmd)
        self.reset_arm()
        self.close_gripper()

    def execute_mission(self,cube_id):
        if(self.target_id == ' ' and cube_id == '1'):
            self.target_id = cube_id
            self.goto(0.0595, 2.7378, 90.0000) # cube number 1
        elif(self.target_id != ' ' and cube_id == '1'):
            self.target_id = cube_id
            self.goto(0.4966, 3.1668, 180.0000) # cube number 1
        elif(cube_id == '2'):
            self.target_id = cube_id
            self.goto(0.1595, 2.7878, -75.0000) # cube number 2
        elif(cube_id == '3'):
            self.target_id = cube_id
            self.goto(1.8032, 2.7603, 0.0000) # cube number 3
        elif(cube_id == '4'):
            self.target_id = cube_id
            self.goto(2.0093, 0.5382, -137.7316) # cube number 4
        elif(cube_id == '5'):
            self.target_id = cube_id
            self.goto(2.7125, -0.7569, 0.0000) # cube number 5
        self.grasp_cube()
            
if __name__ == "__main__":
    rospy.init_node('buct_sim2real',anonymous=True)
    r = rospy.Rate(100)
    r.sleep()
    
    tantianwei = buct_sim2real()
    while(1):
        # get missons
        if(tantianwei.state == 0):
            tantianwei.goto_get_mission(0.2017, 1.6649, -4.200)
            tantianwei.get_mission()
            while(len(tantianwei.mission_target_id_list)!=3):
                cmd = Twist()
                cmd.linear.x = 0.01*(2*random.random()-1)
                cmd.linear.y = 0.01*(2*random.random()-1)
                cmd.angular.z = 0.01*(2*random.random()-1)
                tantianwei.move_position(cmd)
                tantianwei.get_mission()
            rospy.loginfo(str(tantianwei.mission_target_id_list))
            tantianwei.state = 1

        # grasp and place the first cube
        elif(tantianwei.state == 1):
            tantianwei.execute_mission(tantianwei.mission_target_id_list[0])
            tantianwei.target_id = 'B'
            tantianwei.goto(1.1700, 1.865, 0.0000)
            tantianwei.place_cube()
            tantianwei.state = 2

        # grasp and place the second cube
        elif(tantianwei.state == 2):
            tantianwei.execute_mission(tantianwei.mission_target_id_list[1])
            tantianwei.target_id = 'O'
            tantianwei.goto(1.1700, 1.7375, 0.0000)
            tantianwei.place_cube()
            tantianwei.state = 3

        # grasp and place the third cube
        elif(tantianwei.state == 3):
            tantianwei.execute_mission(tantianwei.mission_target_id_list[2])
            tantianwei.target_id = 'X'
            tantianwei.goto(1.1700, 1.6000, 0.0000)
            tantianwei.move_base_goal = [1.1700, 1.6000, 0.0000]
            tantianwei.place_cube()
            tantianwei.state == 4
            exit()

    while not rospy.is_shutdown():
        r.sleep()
