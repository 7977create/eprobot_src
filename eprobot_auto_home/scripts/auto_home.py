#!/usr/bin/python
# coding=gbk
# Copyright 2019 Wechange Tech.
# Developer: FuZhi, Liu (liu.fuzhi@wechangetech.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os
import time
import rospy, math
import actionlib
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class auto_home:
    def __init__(self):
        self.bat_low_times = 0
        self.bat_topic = rospy.get_param('~battery_topic', 'battery') 
        self.bat_min = rospy.get_param('~battery_min', 11.0) 
        self.pos_x = rospy.get_param('~target_pose_x', 0.0) 
        self.pos_y = rospy.get_param('~target_pose_y', 0.0) 
        self.pos_z = rospy.get_param('~target_pose_z', 0.0) 
        self.ori_x = rospy.get_param('~target_ori_x', 0.0) 
        self.ori_y = rospy.get_param('~target_ori_y', 0.0) 
        self.ori_z = rospy.get_param('~target_ori_z', 0.0) 
        self.ori_w = rospy.get_param('~target_ori_w', 0.0) 
        
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.odom_subsciber = rospy.Subscriber("/odom", Odometry, self._get_info)

        self.info = Odometry()
        self.pos_diff = 10.0
        self.last_pose = Vector3()
        self.last_pose.x = 0.0
        self.last_pose.y = 0.0
        self.curr_time = 0.0
        self.gohome = 0
        
        
        
        self.client.wait_for_server()
        self.bat_sub = rospy.Subscriber(self.bat_topic, BatteryState, self.batCB, queue_size=20)
    
    def _get_info(self, msg):

        self.info = msg
        
        self.pos_diff = abs(self.last_pose.x - self.info.pose.pose.position.x) + abs(self.last_pose.y - self.info.pose.pose.position.y)
        
        self.last_pose.x = self.info.pose.pose.position.x
        self.last_pose.y = self.info.pose.pose.position.y       

    def _goal_pose(self):

        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = self.pos_x
        self.goal.target_pose.pose.position.y = self.pos_y
        self.goal.target_pose.pose.position.z = self.pos_z
        self.goal.target_pose.pose.orientation.x = self.ori_x
        self.goal.target_pose.pose.orientation.y = self.ori_y
        self.goal.target_pose.pose.orientation.z = self.ori_z
        self.goal.target_pose.pose.orientation.w = self.ori_w
    
    def send_goal(self):

        self._goal_pose()
        self.client.send_goal(self.goal)
    
    

    def batCB(self,data):
        if data.voltage < self.bat_min and self.bat_low_times < 20 and self.gohome == 0:
            self.bat_low_times += 1
        elif self.bat_low_times == 20:
            self.send_goal()
            self.bat_low_times = 0
            self.gohome = 1
            rospy.loginfo('[eprobot_auto_home]->Get battery data: Voltage=%s',data.voltage)
            rospy.loginfo('[eprobot_auto_home]->battery Low！Go home!')

if __name__  == '__main__':

    try:
        rospy.init_node('eprobot_auto_home')
        rospy.loginfo('auto_home start...')  
        ah = auto_home()         
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down eprobot_auto_home")
        exit(0)