#! /usr/bin/env python2
# -*- coding: utf-8 -*-
# 
# Python2 Python 2 Python2 重要的事情说三遍！
# ROS Melodic 不支持 Python3，使用 Python3 调用 ROS 提供的 API 有可能报错（如本程序）

#from __future__ import print_function

import os
import time
#from interactive import show, get_bool_ans, get_str_ans, save
#from initialize import init, uninit
#from ros_module import ROSNavNode

from geometry_msgs.msg import Twist, Vector3
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
#import geometry_msgs/PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

#import re
import rospy
#import sys
import math
import serial
import string

import actionlib

from std_msgs.msg import  Float64
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class MOVE_ARRIVE:
    def __init__(self):
        #Get params

        self.teb_vel_topic= rospy.get_param('~local_vel_topic','/cmd_vel_local')
        self.plan_vel_topic= rospy.get_param('~plan_vel_topic','/cmd_vel_plan')
        self.cmd_vel_topic= rospy.get_param('~cmd_vel_topic','/cmd_vel')
        
        self.flag = 0
        self.sendgoal_flag = 1
        
        self.px = 0.0
        self.py = 0.0
        self.pz = 0.0
        
        self.px5 = 0.0
        self.py5 = 0.0
        self.pz5 = 0.0

        self.this_pose_x = 0
        self.this_pose_y = 0
        self.this_pose_z = 0
        
        self.trans_x = 0.0
        self.trans_y = 0.0
        self.rotat_z = 0.0

        self.local_trans_x = 0.0
        self.local_trans_y = 0.0
        self.local_rotat_z = 0.0

        self.plan_trans_x = 0.0
        self.plan_trans_y = 0.0
        self.plan_rotat_z = 0.0
        
        self.color = 0

        self.amcl_subsciber = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self._pose_info)
        
        self.local_vel_sub = rospy.Subscriber(self.teb_vel_topic,Twist,self.cmdlocalCB,queue_size=20)
        self.plan_vel_sub = rospy.Subscriber(self.plan_vel_topic,Twist,self.cmdplanCB,queue_size=20)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal",PoseStamped,queue_size=10)
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic,Twist,queue_size=10)
        
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        time.sleep(2.2)

    def cmdlocalCB(self,data):
        self.teb_trans_x = data.linear.x  #*6
        self.teb_trans_y = data.linear.y
        self.teb_rotat_z = data.angular.z
        
        # vel_msg = Twist()
        # vel_msg.linear.x = self.local_trans_x
        # vel_msg.linear.y = self.local_trans_y
        # vel_msg.angular.z = self.local_rotat_z
        # self.cmd_vel_pub.publish(vel_msg)
        
        
        # vel_msg = Twist()
        # if self.color == 1:
            # vel_msg.linear.x = self.local_trans_x
            # vel_msg.linear.y = self.local_trans_y
            # vel_msg.angular.z = self.local_rotat_z
            # self.cmd_vel_pub.publish(vel_msg)
            
        # elif self.color == 0:
            # vel_msg.linear.x = self.plan_trans_x
            # vel_msg.linear.y = self.plan_trans_y
            # vel_msg.angular.z = self.plan_rotat_z
            # self.cmd_vel_pub.publish(vel_msg)
        
    def cmdplanCB(self,data):
        self.plan_trans_x = data.linear.x  #*6
        self.plan_trans_y = data.linear.y
        self.plan_rotat_z = data.angular.z
        
        vel_msg = Twist()
        vel_msg.linear.x = self.plan_trans_x
        vel_msg.linear.y = self.plan_trans_y
        vel_msg.angular.z = self.plan_rotat_z
        self.cmd_vel_pub.publish(vel_msg)
        
    def _goal_pose1(self):
        self.px = -2.300
        self.py = -0.559
        self.pz = 1.749
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = self.px
        self.goal.target_pose.pose.position.y = self.py
        self.goal.target_pose.pose.position.z = 0
        self.goal.target_pose.pose.orientation.x = 0
        self.goal.target_pose.pose.orientation.y = 0
        self.goal.target_pose.pose.orientation.z = math.cos(self.pz/2)
        self.goal.target_pose.pose.orientation.w = math.sin(self.pz/2)
        
        self.goal_msg = PoseStamped()
        self.goal_msg.header.frame_id = 'map'
        self.goal_msg.pose.position.x = self.px
        self.goal_msg.pose.position.y = self.py
        self.goal_msg.pose.position.z = 0
        self.goal_msg.pose.orientation.x = 0
        self.goal_msg.pose.orientation.y = 0
        self.goal_msg.pose.orientation.z = math.cos(self.pz/2)
        self.goal_msg.pose.orientation.w = math.sin(self.pz/2)
        
    def _goal_pose2(self):
        self.px = -0.177
        self.py = -1.246
        self.pz = -3.06
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = self.px
        self.goal.target_pose.pose.position.y = self.py
        self.goal.target_pose.pose.position.z = 0
        self.goal.target_pose.pose.orientation.x = 0
        self.goal.target_pose.pose.orientation.y = 0
        self.goal.target_pose.pose.orientation.z = math.cos(self.pz/2)
        self.goal.target_pose.pose.orientation.w = math.sin(self.pz/2)
        
        self.goal_msg = PoseStamped()
        self.goal_msg.header.frame_id = 'map'
        self.goal_msg.pose.position.x = self.px
        self.goal_msg.pose.position.y = self.py
        self.goal_msg.pose.position.z = 0
        self.goal_msg.pose.orientation.x = 0
        self.goal_msg.pose.orientation.y = 0
        self.goal_msg.pose.orientation.z = math.cos(self.pz/2)
        self.goal_msg.pose.orientation.w = math.sin(self.pz/2)
        
    def _goal_pose3(self):
        self.px = 2.132
        self.py = -0.621
        self.pz = -1.467
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = self.px
        self.goal.target_pose.pose.position.y = self.py
        self.goal.target_pose.pose.position.z = 0
        self.goal.target_pose.pose.orientation.x = 0
        self.goal.target_pose.pose.orientation.y = 0
        self.goal.target_pose.pose.orientation.z = math.cos(self.pz/2)
        self.goal.target_pose.pose.orientation.w = math.sin(self.pz/2)
        
        self.goal_msg = PoseStamped()
        self.goal_msg.header.frame_id = 'map'
        self.goal_msg.pose.position.x = self.px
        self.goal_msg.pose.position.y = self.py
        self.goal_msg.pose.position.z = 0
        self.goal_msg.pose.orientation.x = 0
        self.goal_msg.pose.orientation.y = 0
        self.goal_msg.pose.orientation.z = math.cos(self.pz/2)
        self.goal_msg.pose.orientation.w = math.sin(self.pz/2)
        
    def _goal_pose4(self):
        self.px = 0
        self.py = 0
        self.pz = 0.001
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = self.px
        self.goal.target_pose.pose.position.y = self.py
        self.goal.target_pose.pose.position.z = 0
        self.goal.target_pose.pose.orientation.x = 0
        self.goal.target_pose.pose.orientation.y = 0
        self.goal.target_pose.pose.orientation.z = math.cos(self.pz/2)
        self.goal.target_pose.pose.orientation.w = math.sin(self.pz/2)
        
        self.goal_msg = PoseStamped()
        self.goal_msg.header.frame_id = 'map'
        self.goal_msg.pose.position.x = self.px
        self.goal_msg.pose.position.y = self.py
        self.goal_msg.pose.position.z = 0
        self.goal_msg.pose.orientation.x = 0
        self.goal_msg.pose.orientation.y = 0
        self.goal_msg.pose.orientation.z = math.cos(self.pz/2)
        self.goal_msg.pose.orientation.w = math.sin(self.pz/2)
    
    def _goal_pose5(self):
        self.px = -0.1
        self.py = -1.581
        self.pz = 0.01
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = self.px
        self.goal.target_pose.pose.position.y = self.py
        self.goal.target_pose.pose.position.z = 0
        self.goal.target_pose.pose.orientation.x = 0
        self.goal.target_pose.pose.orientation.y = 0
        self.goal.target_pose.pose.orientation.z = math.cos(self.pz/2)
        self.goal.target_pose.pose.orientation.w = math.sin(self.pz/2)
        
        self.goal_msg = PoseStamped()
        self.goal_msg.header.frame_id = 'map'
        self.goal_msg.pose.position.x = self.px
        self.goal_msg.pose.position.y = self.py
        self.goal_msg.pose.position.z = 0
        self.goal_msg.pose.orientation.x = 0
        self.goal_msg.pose.orientation.y = 0
        self.goal_msg.pose.orientation.z = math.cos(self.pz/2)
        self.goal_msg.pose.orientation.w = math.sin(self.pz/2)
    
    def _pose_info(self, data):
        self.this_pose_x = data.pose.pose.position.x
        self.this_pose_y = data.pose.pose.position.y
        self.this_pose_z = data.pose.pose.position.z
        
        #print('pose_info')
    
    
    def send_goal1(self):
        self._goal_pose1()
        self.client.send_goal(self.goal)
        self.goal_pub.publish(self.goal_msg) 
        print('发点一')
    def send_goal2(self):
        self._goal_pose2()
        self.client.send_goal(self.goal)
        self.goal_pub.publish(self.goal_msg) 
        print('发点二')
    def send_goal3(self):
        self._goal_pose3()
        self.client.send_goal(self.goal)
        self.goal_pub.publish(self.goal_msg) 
        print('发点三')
    def send_goal4(self):
        self._goal_pose4()
        self.client.send_goal(self.goal)
        self.goal_pub.publish(self.goal_msg) 
        print('发点四')
    def send_goal5(self):
        self._goal_pose5()
        self.client.send_goal(self.goal)
        self.goal_pub.publish(self.goal_msg) 
        print('发点五')
                                   
    def sendgoalCB(self):
        lim_x1 = -1.8
        max_x1 = -1
        lim_y1 = -1.45
        max_y1 = -0.9
        lim_x2 = 0.6
        max_x2 = 1.3
        lim_y2 = -1.45
        max_y2 = -0.9
        lim_x3 = 1
        max_x3 = 1.7
        lim_y3 = -0.3
        max_y3 = 0.3
        lim_x4 = -1.6
        max_x4 = 0.8
        lim_y4 = -0.3
        max_y4 = 0.3
        
        self.send_goal3()
        self.sendgoal_flag = 2

      

        ''' for i in range(10):
            while True:
                if self.this_pose_x >= lim_x1 and self.this_pose_x <= max_x1 and self.this_pose_y >= lim_y1 and self.this_pose_y <= max_y1 :
                    self.send_goal2()
                    time.sleep(0.02)
                    print('发了第二个目标点')
                if self.this_pose_x >= lim_x2 and self.this_pose_x <= max_x2 and self.this_pose_y >= lim_y2 and self.this_pose_y <= max_y2 :
                    self.send_goal3()
                    time.sleep(0.02)
                    print('发了第三个目标点')
                if self.this_pose_x >= lim_x3 and self.this_pose_x <= max_x3 and self.this_pose_y >= lim_y3 and self.this_pose_y <= max_y3 :
                    self.send_goal4()
                    time.sleep(0.02)
                    print('发了第四个目标点')
                if self.this_pose_x >= lim_x4 and self.this_pose_x <= max_x4 and self.this_pose_y >= lim_y4 and self.this_pose_y <= max_y4 :
                    self.send_goal1()
                    time.sleep(0.02)
                    #break
            i += 1'''
            
        # for i in range(10):
            # while not rospy.is_shutdown():
                # if self.this_pose_x >= lim_x1 and self.this_pose_x <= max_x1 and self.this_pose_y >= lim_y1 and self.this_pose_y <= max_y1 :
                    # self.send_goal4()
                    # time.sleep(0.02)
                    # print('发了第二个目标点')
                # if self.this_pose_x >= lim_x2 and self.this_pose_x <= max_x2 and self.this_pose_y >= lim_y2 and self.this_pose_y <= max_y2 :
                    # self.send_goal1()
                    # time.sleep(0.02)
                    # print('发了第三个目标点')
                # if self.this_pose_x >= lim_x3 and self.this_pose_x <= max_x3 and self.this_pose_y >= lim_y3 and self.this_pose_y <= max_y3 :
                    # self.send_goal2()
                    # time.sleep(0.02)
                    # print('发了第四个目标点')
                # if self.this_pose_x >= lim_x4 and self.this_pose_x <= max_x4 and self.this_pose_y >= lim_y4 and self.this_pose_y <= max_y4 :
                    # self.send_goal3()
                    # time.sleep(0.02)
                    ##break
            # i += 1
            
        for i in range(2): 
            while not (rospy.is_shutdown() or self.flag):
                if self.color == 1:
                    if self.sendgoal_flag == 4 and self.this_pose_x >= lim_x1 and self.this_pose_x <= max_x1 and self.this_pose_y >= lim_y1 and self.this_pose_y <= max_y1 :
                        self.send_goal4()
                        self.sendgoal_flag = 1
                        time.sleep(0.1)
                        self.flag = 1
                    if self.sendgoal_flag == 3 and self.this_pose_x >= -0.5 and self.this_pose_x <= 0.4: #and self.this_pose_y >= -0.75 and self.this_pose_y <= -0.5 :
                        self.send_goal1()
                        self.sendgoal_flag = 4
                        self.color = 0
                        time.sleep(0.1)
                    if self.sendgoal_flag == 2 and self.this_pose_x >= lim_x3 and self.this_pose_x <= max_x3 and self.this_pose_y >= lim_y3 and self.this_pose_y <= max_y3 :
                        self.send_goal5()
                        self.sendgoal_flag = 3
                        time.sleep(15)
                        #if self.this_pose_x == self.px5 and self.this_pose_y == self.py5 and self.this_pose_z == self.pz5 :#第5个目标点附近xy小阈值
                        # if self.this_pose_x <= 0.15:
                            # self.color = 0
                        
                        
                    if self.sendgoal_flag == 1 and self.this_pose_x >= lim_x4 and self.this_pose_x <= max_x4 and self.this_pose_y >= lim_y4 and self.this_pose_y <= max_y4 :
                        self.send_goal3()
                        self.sendgoal_flag = 2
                        time.sleep(0.1)
                        
                    # if self.sendgoal_flag == 4 and self.this_pose_x >= lim_x1 and self.this_pose_x <= max_x1 and self.this_pose_y >= lim_y1 and self.this_pose_y <= max_y1 :
                        # self.send_goal4()
                        # self.sendgoal_flag = 1
                        # time.sleep(0.1)
                        # self.flag = 1
                        # print('发了第二个目标点')
                    # if self.sendgoal_flag == 3 and self.this_pose_x >= lim_x2 and self.this_pose_x <= max_x2 and self.this_pose_y >= lim_y2 and self.this_pose_y <= max_y2 :
                        # self.send_goal1()
                        # self.send_goal1()
                        # self.sendgoal_flag = 4
                        # time.sleep(0.1)
                        # print('发了第三个目标点')
                    # if self.sendgoal_flag == 2 and self.this_pose_x >= lim_x3 and self.this_pose_x <= max_x3 and self.this_pose_y >= lim_y3 and self.this_pose_y <= max_y3 :
                        # self.send_goal2()
                        # self.sendgoal_flag = 3
                        # time.sleep(25)
                        # if self.this_pose_x == self.px5 and self.this_pose_y == self.py5 and self.this_pose_z == self.pz5 :#第5个目标点附近xy小阈值
                            # self.color = 0
                        # print('发了第四个目标点')
                    # if self.sendgoal_flag == 1 and self.this_pose_x >= lim_x4 and self.this_pose_x <= max_x4 and self.this_pose_y >= lim_y4 and self.this_pose_y <= max_y4 :
                        # self.send_goal3()
                        # self.sendgoal_flag = 2
                        # time.sleep(0.1)
                        
                else: 
                    if self.sendgoal_flag == 4 and self.this_pose_x >= lim_x1 and self.this_pose_x <= max_x1 and self.this_pose_y >= lim_y1 and self.this_pose_y <= max_y1 :
                        self.send_goal4()
                        self.sendgoal_flag = 1
                        time.sleep(0.1)
                        self.flag = 1
                    if self.sendgoal_flag == 3 and self.this_pose_x >= lim_x2 and self.this_pose_x <= max_x2 and self.this_pose_y >= lim_y2 and self.this_pose_y <= max_y2 :
                        self.send_goal1()
                        self.sendgoal_flag = 4
                        time.sleep(0.1)
                    if self.sendgoal_flag == 2 and self.this_pose_x >= lim_x3 and self.this_pose_x <= max_x3 and self.this_pose_y >= lim_y3 and self.this_pose_y <= max_y3 :
                        self.send_goal2()
                        self.sendgoal_flag = 3
                        time.sleep(0.1)
                    if self.sendgoal_flag == 1 and self.this_pose_x >= lim_x4 and self.this_pose_x <= max_x4 and self.this_pose_y >= lim_y4 and self.this_pose_y <= max_y4 :
                        self.send_goal3()
                        self.sendgoal_flag = 2
                        time.sleep(0.1)
                        
            if i == 0:
                self.color = 1
            self.flag = 0
            
        
        
        
#main function
if __name__=="__main__":
    try:
        rospy.init_node('navigation_target_instruction',anonymous=True)
        rospy.loginfo('navigation_target instruction start...')
        MA = MOVE_ARRIVE()
        MA.sendgoalCB()
        rospy.spin()
    except KeyboardInterrupt:
        #ma.serial.close
        print("Shutting down")


#2341