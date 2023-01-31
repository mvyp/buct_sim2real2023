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
import random

class buct_sim2real:
    def __init__(self):
        self.state = 0
        self.gama_x = 0.015
        self.gama_y = 0.015
        self.min_vel = 0.20
        self.goal = []
        self.goal_grasp = [0.045, 0.0, 0.25]
        self.goal_place = [0.045, 0.0, 0.15]
        self.id = ' '
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
            self.tf_listener.waitForTransform('/map', '/grasp_position1', rospy.Time(), rospy.Duration(2.0))
            self.tf_listener.waitForTransform('/map', '/grasp_position2', rospy.Time(), rospy.Duration(2.0))
            self.tf_listener.waitForTransform('/map', '/grasp_position3', rospy.Time(), rospy.Duration(2.0))
            self.tf_listener.waitForTransform('/map', '/grasp_position4', rospy.Time(), rospy.Duration(2.0))
            self.tf_listener.waitForTransform('/map', '/grasp_position0', rospy.Time(), rospy.Duration(2.0))
            self.tf_listener.waitForTransform('/map', '/camera_color_optical_frame', rospy.Time(), rospy.Duration(2.0))
            self.tf_listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(2.0))
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
            (trans1, rot1) = self.tf_listener.lookupTransform('/map','/grasp_position'+self.id, rospy.Time(0))
            (trans2, rot2) = self.tf_listener.lookupTransform('/map','/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf Error")
            self.recovery()
            return None 
        cmd_stop=Twist()
        self.move_vel_pub.publish(cmd_stop)
        if(trans1 == self.trans1_cache and trans2 == self.trans2_cache and rot1 ==self.rot1_cache and rot2 ==self.rot2_cache):
            return None
        else:
            self.trans1_cache = trans1
            self.trans2_cache = trans2
            self.rot1_cache = rot1
            self.rot2_cache = rot2
            return trans1, rot1, trans2, rot2

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
        rospy.loginfo("status:")
        rospy.loginfo(status)
        rospy.loginfo("result:")
        rospy.loginfo(result)

    def active_cb(self):
        rospy.loginfo("navigation has be actived")

    def feedback_cb(self, feedback):
        pass

    def check_grasp_succeed(self):
        
        if len(self.target_list.target_list) >= 2:
            for target in self.target_list.target_list:
                if target.id == self.id and target.pose.position.z>=0.20: #may need to be changed
                    rospy.loginfo("Fail to grasp,try again!")
                    return False
                else:
                    rospy.loginfo("grasp_succeed")
                    return True
        else:
            rospy.loginfo("grasp_succeed")
            return True
        
    def goto(self, x, y, th):
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
                rospy.loginfo("GOTO reach goal %s succeeded!"%p)
        
        return True
    def goto_grasp(self, x, y, th):
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
        rospy.sleep(2)
        while(1):
            
            if (self.cancel_goal()==True):
                rospy.loginfo("already find the target")
                self.move_base.cancel_goal()
                break
            else:
                rospy.loginfo(self.id)
            rospy.sleep(0.5)

        result = self.move_base.wait_for_result(rospy.Duration(15))  #?
        
        return True

    def moving_to_point(self):
        t1 = []
        t2 = []
        r1 = []
        r2 = []
        failed = 0
        rospy.sleep(0.7)
        try:
            t1,r1,t2,r2 = self.get_position()
            rospy.loginfo('arrive at the grasp position')
        except:
            rospy.loginfo('failed to get grasp position')
        if(len(t1)==3 and len(t2)==3 and len(r1)==4 and len(r2)==4):
            e1 = transformations.euler_from_quaternion(r1)
            e2 = transformations.euler_from_quaternion(r2)
            th1 = e1[2] / pi * 180
            th2 = e2[2] / pi * 180
            rospy.loginfo('goto_point: '+str(t1[0])+' '+str(t1[1])+' '+str(th1))
            self.goto(t1[0], t1[1], th1)

    def get_mission(self):
        # read
        try:
            for target in self.target_list.target_list:
                if(target.id=='O'):
                    target.id='3'
                if (len(self.mission_target_list)<3 and (target.id not in self.mission_target_id_list) and (target.pose.position.y<=-0.1)  and (target.id!='B') and (target.id!='X')):
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
                roll,pitch,yaw = self.euler_from_quaternion(i.pose.orientation.x,i.pose.orientation.y,i.pose.orientation.z,i.pose.orientation.w)
                #rospy.loginfo('r: '+str(roll*180/pi)+' '+'p: '+str(pitch*180/pi)+' '+'y: '+str(yaw*180/pi))
                if(i.id == self.id and i.pose.position.y>=-0.025 and i.pose.position.y<=0.025 and abs(roll*180/pi)>=170):
                    q0 = i.pose.orientation.w
                    q1 = i.pose.orientation.x
                    q2 = i.pose.orientation.y
                    q3 = i.pose.orientation.z

                    r00 = 2 * (q0 * q0 + q1 * q1) - 1
                    r01 = 2 * (q1 * q2 - q0 * q3)
                    r02 = 2 * (q1 * q3 + q0 * q2)
                    r10 = 2 * (q1 * q2 + q0 * q3)
                    r11 = 2 * (q0 * q0 + q2 * q2) - 1
                    r12 = 2 * (q2 * q3 - q0 * q1)
                    r20 = 2 * (q1 * q3 - q0 * q2)
                    r21 = 2 * (q2 * q3 + q0 * q1)
                    r22 = 2 * (q0 * q0 + q3 * q3) - 1

                    h1 = np.array([[r00, r01, r02,i.pose.position.x],[r10, r11, r12,i.pose.position.y],[r20, r21, r22,i.pose.position.z],[0,0,0,1]])
                    h2 = np.array([[1,0,0,0],[0,1,0,0],[0,0,0,-0.0225],[0,0,0,1]])
                    h = np.matmul(h1,h2)
                    # h is the homogeneous transform matrix of the inner centeral point of the box
                    # it corresponds to the transform from camera_optical_fram to the inner central point of the box
                    target_z += h[2][3]
                    target_x += h[0][3]
                    length += 1
                    obj_tf = tf.TransformBroadcaster()
                    obj_tf.sendTransform((h[0][3],h[1][3],h[2][3]),(0,0,-0.707,0.707),rospy.Time.now(),'abba','camera_color_optical_frame')
        # get mean value
        target_z /= length
        target_x /= length
        
        distance_in_x = target_z - self.goal[2]
        distance_in_y = target_x - self.goal[0]
        #rospy.loginfo('x: '+str(distance_in_x)+' y: '+str(distance_in_y))
        return distance_in_x, distance_in_y
    
    def adjust_pose(self,mode):
        cmd = Twist()
        cmd_stop = Twist()
        distance_in_x, distance_in_y = self.update_distance(mode)
        while(abs(distance_in_x) > self.gama_x or abs(distance_in_y) > self.gama_y):
            #rospy.loginfo('x: '+str(distance_in_x)+' y: '+str(distance_in_y))
            if (abs(distance_in_y) > self.gama_y and abs(distance_in_x) > self.gama_x):
                #rospy.loginfo('1')
                cmd.linear.y = -distance_in_y/abs(distance_in_y)*self.min_vel
                cmd.linear.x = distance_in_x/abs(distance_in_x)*self.min_vel
            elif(abs(distance_in_y) <= self.gama_y and abs(distance_in_x) > self.gama_x):
                #rospy.loginfo('2')
                cmd.linear.y = 0
                cmd.linear.x = distance_in_x/abs(distance_in_x)*self.min_vel
            elif(abs(distance_in_y) > self.gama_y and abs(distance_in_x) <= self.gama_x):
                #rospy.loginfo('3')
                cmd.linear.y = -distance_in_y/abs(distance_in_y)*self.min_vel
                cmd.linear.x = 0
            elif(abs(distance_in_y) <= self.gama_y and abs(distance_in_x) <= self.gama_x):
                #rospy.loginfo('4')
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
        self.moving_to_point()
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
    def ifthere(self):
        rospy.loginfo("check whether in the list")
        for i in self.target_list.target_list:
            if(i.id == self.id):
                rospy.loginfo("target in the list")
                return True
        rospy.loginfo(" target NOT in the list")
        return False

    def place_cube(self):
        cmd_stop=Twist()
        self.move_vel_pub.publish(cmd_stop)
         
        if(self.ifthere()==True):
            self.adjust_pose('place')
            self.reset_arm()
            self.move_arm0()
            position_cmd = Twist()
            position_cmd.linear.x = 0.15 #0.2
            self.move_position_pub.publish(position_cmd)
            rospy.sleep(0.7)
            self.move_arm()
            rospy.sleep(1.0)
            self.open_gripper()
            rospy.sleep(0.1)
            position_cmd.linear.x = -0.2
            self.move_position_pub.publish(position_cmd)
            self.reset_arm()
            self.close_gripper()


        else:
            rospy.loginfo("target not find")#  test the problem of stopping in the front of BOX
            rospy.loginfo("!backward recovery begin!")
            self.back_recovery()

    def execute_mission(self,cube_id):
        if(cube_id == '1'):
            self.id = cube_id
            self.goto_grasp(0.5966, 3.1668, 180.0000) # cube number 1
        elif(cube_id == '2'):
            self.id = cube_id
            self.goto_grasp(0.0095, 2.6878, -55.0000) # cube number 2
        elif(cube_id == '3'):
            self.id = cube_id
            self.goto_grasp(1.7032, 2.7603, 0.0000) # cube number 3
        elif(cube_id == '4'):
            self.id = cube_id
            self.goto_grasp(2.0093, 0.5382, -137.7316) # cube number 4
        elif(cube_id == '5'):
            self.id = cube_id
            self.goto_grasp(2.7125, -0.7569, 0.0000) # cube number 5      
        self.grasp_cube()
        if (not self.check_grasp_succeed()==True): 
            rospy.loginfo("FAIL to grasp,try again")
            self.grasp_cube()
    def cancel_goal(self):
        for target in self.target_list.target_list:
                if (target.id == self.id):
                    return True
                else:
                    return False      
    def recovery(self):
        rospy.loginfo("!recovery begin!")
        cmd_rotate_r=Twist()
        cmd_rotate_r.angular.z=-0.5
        self.move_vel_pub.publish(cmd_rotate_r)
        rospy.sleep(0.2)
        self.get_position()

    def back_recovery(self):
        cmd_rotate_back=Twist()
        cmd_rotate_back.linear.x=-0.2
        self.move_vel_pub.publish(cmd_rotate_back)
        rospy.sleep(0.2)
        self.place_cube()

            
if __name__ == "__main__":
    rospy.init_node('buct_sim2real',anonymous=True)
    r = rospy.Rate(100)
    r.sleep()
    primary_list = ['3','4','5','1','2']  #define a best way to grasp
    resort_state = [1,2,3]
    current_state = 0
    tantianwei = buct_sim2real()
    while(1):
        # get missons
        if(tantianwei.state == 0):
            tantianwei.goto(0.2017, 1.6649, -4.200)
            tantianwei.get_mission()
            if(len(tantianwei.mission_target_id_list)!=3):
                cmd = Twist()
                cmd.linear.x = 0.01*(2*random.random()-1)
                cmd.linear.y = 0.01*(2*random.random()-1)
                cmd.angular.z = 0.01*(2*random.random()-1)
                tantianwei.move_position(cmd)
                tantianwei.get_mission()
            rospy.loginfo(str(tantianwei.mission_target_id_list))

            break_f = False
            for i in primary_list:      #change the 'mission_target_id_list'
                for j in range(len(tantianwei.mission_target_id_list)):
                    if tantianwei.mission_target_id_list[j] == i:
                        resort_state.pop(j)
                        resort_state.insert(0,(j+1))
                        break_f = True
                        break
                if break_f:
                    break

            print(resort_state)
            tantianwei.state = resort_state[current_state]

        # grasp and place the first cube
        elif(tantianwei.state == 1):
            current_state += 1
            tantianwei.execute_mission(tantianwei.mission_target_id_list[0])
            tantianwei.goto(1.1200, 1.8250, 0.0000)
            tantianwei.id = 'B'
            tantianwei.place_cube()
            tantianwei.state = resort_state[current_state]

        # grasp and place the second cube
        elif(tantianwei.state == 2):
            current_state += 1
            tantianwei.execute_mission(tantianwei.mission_target_id_list[1])
            tantianwei.goto(1.1200, 1.7375, 0.0000)
            tantianwei.id = 'O'
            tantianwei.place_cube()
            tantianwei.state = resort_state[current_state]

        # grasp and place the third cube
        elif(tantianwei.state == 3):
            current_state += 1
            tantianwei.execute_mission(tantianwei.mission_target_id_list[2])
            tantianwei.goto(1.1200, 1.6000, 0.0000)
            tantianwei.id = 'X'
            tantianwei.place_cube()
            tantianwei.state == resort_state[current_state]


        elif(current_state == 4):
            break
    exit()
    while not rospy.is_shutdown():
        r.sleep()