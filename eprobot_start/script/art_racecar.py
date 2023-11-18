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
import re
import rospy
import tf
import time
import sys
import math
import serial
import string
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Temperature
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32
from std_msgs.msg import Float64                                                                                                 
import ctypes


class raspi_getInfo:
    def __init__(self):
        self.sta = 0
    
    # Return CPU temperature as a character string                                      
    def getCPUtemperature(self):
        res = os.popen('cat /sys/class/thermal/thermal_zone0/temp').readline()
        return(str(float(res.replace("temp=","").replace("'C\n",""))/1000.0))
 
    # Return RAM information (unit=kb) in a list                                       
    # Index 0: total RAM                                                               
    # Index 1: used RAM                                                                 
    # Index 2: free RAM                                                                 
    def getRAMinfo(self):
        p = os.popen('free')
        i = 0
        while 1:
            i = i + 1
            line = p.readline()
            if i==2:
                return(line.split()[1:4])
 
    # Return % of CPU used by user as a character string                                
    def getCPUuse(self):
        return(str(os.popen("top -n1 | awk '/Cpu\(s\):/ {print $2}'").readline().strip()))
 
    # Return information about disk space as a list (unit included)                     
    # Index 0: total disk space                                                         
    # Index 1: used disk space                                                         
    # Index 2: remaining disk space                                                     
    # Index 3: percentage of disk used                                                  
    def getDiskSpace(self):
        p = os.popen("df -h /")
        i = 0
        while 1:
            i = i +1
            line = p.readline()
            if i==2:
                return(line.split()[1:5])
    
    # use os.popen() to get ip
    def getRaspiIPInfo(self):
        cmd_file = os.popen('ifconfig wlan0')
        cmd_result = cmd_file.read()
        pattern = re.compile(r'(inet )(\d{1,3}.\d{1,3}.\d{1,3}.\d{1,3})')
        ip_wlan0 = re.findall(pattern, cmd_result)
        cmd_file = os.popen('ifconfig eth0')
        cmd_result = cmd_file.read()
        ip_eth0 = re.findall(pattern, cmd_result)
        cmd_file = os.popen('iwconfig wlan0')
        cmd_result = cmd_file.read()
        pattern = re.compile(r'(Mode:)(\w{1,7})')
        wlan0_mode = re.findall(pattern, cmd_result)
        if len(ip_wlan0) == 0:
            ip_wlan0_list=['0','0','0','0'] 
        else:
            ip_wlan0_list=ip_wlan0[0][1].split('.')
        if len(ip_eth0) == 0:
            ip_eth0_list=['0','0','0','0'] 
        else:
            ip_eth0_list=ip_eth0[0][1].split('.')
        list_all =  ip_wlan0_list + ip_eth0_list
        list_all.insert(0,wlan0_mode[0][1])
        return list_all

#class queue is design for uart receive data cache
class queue:
    def __init__(self, capacity = 1024*4):
        self.capacity = capacity
        self.size = 0
        self.front = 0
        self.rear = 0
        self.array = [0]*capacity
 
    def is_empty(self):
        return 0 == self.size
 
    def is_full(self):
        return self.size == self.capacity
 
    def enqueue(self, element):
        if self.is_full():
            raise Exception('queue is full')
        self.array[self.rear] = element
        self.size += 1
        self.rear = (self.rear + 1) % self.capacity
 
    def dequeue(self):
        if self.is_empty():
            raise Exception('queue is empty')
        self.size -= 1
        self.front = (self.front + 1) % self.capacity
 
    def get_front(self):
        return self.array[self.front]
    
    def get_front_second(self):
        return self.array[((self.front + 1) % self.capacity)]

    def get_queue_length(self):
        return (self.rear - self.front + self.capacity) % self.capacity

    def show_queue(self):
        for i in range(self.capacity):
            print self.array[i],
        print(' ')
#class BaseControl:
class BaseControl:
    def __init__(self):
        #Get params
        self.baseId = rospy.get_param('~base_id','base_footprint')
        #self.odomId = rospy.get_param('~odom_id','vesc_odom')
        self.odomId = rospy.get_param('~odom_id','odom')
        self.imuId = rospy.get_param('~imu_id','IMU_link')                                                 
        self.device_port = rospy.get_param('~port','/dev/EPRobot_base')
        self.baudrate = int(rospy.get_param('~baudrate','115200'))
        self.odom_freq = int(rospy.get_param('~odom_freq','50'))
        #self.odom_topic = rospy.get_param('~odom_topic','/vesc_odom')
        self.odom_topic = rospy.get_param('~odom_topic','odom')
        self.battery_topic = rospy.get_param('~battery_topic','battery')
        self.RPi_topic = rospy.get_param('~RPi_topic','rpi_info')                       #rpi
        self.battery_freq = float(rospy.get_param('~battery_freq','1'))
        self.cmd_vel_topic= rospy.get_param('~cmd_vel_topic','/cmd_vel')   #need to set from launch
        self.feedback_vel_topic= rospy.get_param('~feedback_vel_topic','/feedback_vel')
        self.wheelbase = rospy.get_param('~wheelbase', 0.335)
        self.imu_topic = rospy.get_param('~imu_topic','/imu_data')#_data
        self.kp = rospy.get_param('~base_kp',4.0)
        self.ki = rospy.get_param('~base_ki',7.0)
        self.kd = rospy.get_param('~base_kd',6.0)
        
        self.kv = rospy.get_param('~base_kv',1.0)
        self.LaserMode = rospy.get_param('~Laser_Mode','Express')
        self.is_pub_odom_tf = rospy.get_param('~is_pub_odom_tf','false')
		
#	self.pub_imu = bool(rospy.get_param('~pub_imu','False'))
#    if(self.pub_imu == True):
#		self.imuId = rospy.get_param('~imu_id','imu')

        self.imu_sub_topic = rospy.get_param('~imu_sub_topic','imu_data')
        
#		self.imu_freq = float(rospy.get_param('~imu_freq','50'))
#	self.sub_ackermann = bool(rospy.get_param('~sub_ackermann','False'))	
        #define param
        self.current_time = rospy.Time.now()
        self.previous_time = self.current_time
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        # serialIDLE_flag: 0=idle ; 4=tx ing ; 
        self.serialIDLE_flag = 0
        self.trans_x = 0.0
        self.trans_y = 0.0
        self.rotat_z = 0.0
        self.speed = 0.0
        self.steering_angle = 0.0
        self.motor_pwm = 0
        self.steering_angle_back = 0
        self.Circleloop = queue(capacity = 1024*4)
        self.Raspi_info = raspi_getInfo()
        self.Vx = 0
        self.Vy = 0
        self.Vyaw = 0
        self.Yawz = 0
        self.Vvoltage = 0
        self.Icurrent = 0
        self.Gyro = [0,0,0]
        self.Accel = [0,0,0]
        self.Quat = [0,0,0,0]
        self.movebase_firmware_version = [0,0,0]
        self.movebase_hardware_version = [0,0,0]
        self.last_cmd_vel_time = rospy.Time.now()

        # Serial Communication
        try:
            self.serial = serial.Serial(self.device_port,self.baudrate,timeout=10)
            rospy.loginfo("Opening Serial")
            try:
                if self.serial.in_waiting:
                    self.serial.readall()
            except:
                rospy.loginfo("Opening Serial Try Faild")
                pass
        except:
            rospy.logerr("Can not open Serial"+self.device_port)
            self.serial.close
            sys.exit(0)
        rospy.loginfo("Serial Open Succeed")

        self.sub = rospy.Subscriber(self.cmd_vel_topic,Twist,self.cmdCB,queue_size=20)
        self.pub = rospy.Publisher(self.odom_topic,Odometry,queue_size=10)
        self.feedback_vel_pub = rospy.Publisher(self.feedback_vel_topic,Twist,queue_size=20)
        self.battery_pub = rospy.Publisher(self.battery_topic,BatteryState,queue_size=3)
        self.imu_pub = rospy.Publisher(self.imu_topic,Imu,queue_size=10)
        self.RPi_pub = rospy.Publisher(self.RPi_topic,BatteryState,queue_size=20)
        #self.sub_imu = rospy.Subscriber(self.imu_topic,Imu,self.imuCB,queue_size=30)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.timer_odom = rospy.Timer(rospy.Duration(1.0/self.odom_freq),self.timerOdomCB)
        self.timer_battery = rospy.Timer(rospy.Duration(1.0/self.battery_freq),self.timerBatteryCB)
        self.timer_communication = rospy.Timer(rospy.Duration(5.0/1000),self.timerCommunicationCB)
        #so need this gap
        time.sleep(2.2)

    def convert_trans_rot_vel_to_steering_angle(self, v, omega, wheelbase):
        if omega == 0 or v == 0:
            return 0
        radius = v / omega
        return math.atan(wheelbase / radius)
    #CRC-8 Calculate
    def crc_1byte(self,data):
        crc_1byte = 0
        for i in range(0,8):
            if((crc_1byte^data)&0x01):
                crc_1byte^=0x18
                crc_1byte>>=1
                crc_1byte|=0x80
            else:
                crc_1byte>>=1
            data>>=1
        return crc_1byte
    def crc_byte(self,data,length):
        ret = 0
        for i in range(length):
            ret = self.crc_1byte(ret^data[i])
        return ret               
    #Subscribe vel_cmd call this to send vel cmd to move base
    def cmdCB(self,data):
        self.trans_x = data.linear.x  #*6
        self.trans_y = data.linear.y
        self.rotat_z = data.angular.z
        if self.trans_x < 0.1 and self.trans_x > 0:
            self.trans_x = 0.1
        if self.trans_x >-0.1 and self.trans_x < 0:
            self.trans_x = -0.1
        steering = self.convert_trans_rot_vel_to_steering_angle(self.trans_x, self.rotat_z, self.wheelbase)
        #if self.trans_x != 0 or self.rotat_z != 0 :
        #    rospy.loginfo('[base control]->Send cmd_val to car: Vx=%s,Steering=%s',self.trans_x,steering)
        #    rospy.loginfo('[base control]->car current vel: Vx=%s',float(ctypes.c_int16(self.Vx).value/1000.0))
        #    rospy.loginfo('[base control]->car info:motor pwm=%s,Steering=%s',ctypes.c_int16(self.motor_pwm).value,self.steering_angle_back)
        self.last_cmd_vel_time = rospy.Time.now()
        outputdata = [0x5a,0x12,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
        outputdata[4] = (int(self.trans_x*1000.0)>>8)&0xff
        outputdata[5] = int(self.trans_x*1000.0)&0xff
        outputdata[6] = (int(self.trans_y*1000.0)>>8)&0xff
        outputdata[7] = int(self.trans_y*1000.0)&0xff
        outputdata[8] = (int(steering*1000.0)>>8)&0xff
        outputdata[9] = int(steering*1000.0)&0xff
        outputdata[10] = (int(self.kp*10.0)>>8)&0xff
        outputdata[11] = int(self.kp*10.0)&0xff
        outputdata[12] = (int(self.ki*10.0)>>8)&0xff
        outputdata[13] = int(self.ki*10.0)&0xff
        outputdata[14] = (int(self.kd*10.0)>>8)&0xff
        outputdata[15] = int(self.kd*10.0)&0xff
        output = chr(0x5a) + chr(18) + chr(0x01) + chr(0x01) + \
            chr((int(self.trans_x*1000.0)>>8)&0xff) + chr(int(self.trans_x*1000.0)&0xff) + \
            chr((int(self.trans_y*1000.0)>>8)&0xff) + chr(int(self.trans_y*1000.0)&0xff) + \
            chr((int(steering*1000.0)>>8)&0xff) + chr(int(steering*1000.0)&0xff) + \
            chr((int(self.kp*10.0)>>8)&0xff) + chr(int(self.kp*10.0)&0xff) + \
            chr((int(self.ki*10.0)>>8)&0xff) + chr(int(self.ki*10.0)&0xff) + \
            chr((int(self.kd*10.0)>>8)&0xff) + chr(int(self.kd*10.0)&0xff) + \
            chr(0x00)
        crc_8 = self.crc_byte(outputdata,len(outputdata)-1)
        output += chr(crc_8)
        while self.serialIDLE_flag:
            time.sleep(0.01)
        self.serialIDLE_flag = 4
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(output)
        except:
            rospy.logerr("Vel Command Send Faild")
        self.serialIDLE_flag = 0 
        
        
        feedback_vel_msg = Twist()
        feedback_vel_msg.linear.x = float(ctypes.c_int16(self.Vx).value/1000.0)
        self.feedback_vel_pub.publish(feedback_vel_msg) 
    
        
        # RPi_msg = BatteryState()
        # RPi_msg.current = float(self.Raspi_info.getCPUtemperature())
        # RPi_msg.percentage = float(self.Raspi_info.getCPUuse())
        # self.RPi_pub.publish(RPi_msg)          
        
    ##Subscribe imu call this to get yaw and Vz
    #def imuCB(self,data):
    #    self.Vyaw = data.angular_velocity.z
    #    x = data.orientation.x
    #    y = data.orientation.y
    #    z = data.orientation.z
    #    w = data.orientation.w
    #    euler = tf.transformations.euler_from_quaternion((x,y,z,w)) 
    #    # self.Yawz = math.atan2(2*(w*z+x*y),1-2*(z*z+y*y))
    #    self.Yawz = euler[2]
    #    rospy.loginfo('[base control]->Get imu data: Vyaw=%s,Yawz=%s',self.Vyaw,self.Yawz)
    ##depend on communication protocol
    def timerCommunicationCB(self,event):
        length = self.serial.in_waiting
        if length:
            reading = self.serial.read_all()
            if len(reading)!=0:
                for i in range(0,len(reading)):
                    data = (int(reading[i].encode('hex'),16)) 
                    try:
                        self.Circleloop.enqueue(data)
                    except:
                        rospy.logerr("Circleloop.enqueue Faild")
        else:
            pass
        if self.Circleloop.is_empty()==False:
            if self.Circleloop.is_empty()==False:
                data = self.Circleloop.get_front()
            else:
                pass
            if data == 0x5a:
                length = self.Circleloop.get_front_second()
                if length > 1 :
                    if self.Circleloop.get_front_second() <= self.Circleloop.get_queue_length():
                        databuf = []
                        for i in range(length):
                            databuf.append(self.Circleloop.get_front())
                            self.Circleloop.dequeue()
                        
                        if (databuf[length-1]) == self.crc_byte(databuf,length-1):
                            pass
                        else:
                            # print databuf
                            # print "Crc check Err %d"%self.crc_byte(databuf,length-1)
                            pass
                        #parse receive data
                        if(databuf[3] == 0x04):
                            self.Vx =    databuf[4]*256
                            self.Vx +=   databuf[5]
                            self.motor_pwm =    databuf[6]*256
                            self.motor_pwm +=   databuf[7]
                            self.steering_angle_back =    databuf[8]*256
                            self.steering_angle_back +=   databuf[9]
                            # self.Vyaw =  databuf[8]*256
                            # self.Vyaw += databuf[9]
                        elif (databuf[3] == 0x08):
                            self.Vvoltage = databuf[4]*256
                            self.Vvoltage += databuf[5]
                            self.Icurrent = databuf[6]*256
                            self.Icurrent += databuf[7]     
                        elif (databuf[3] == 0x0a):
                            self.Vx =    databuf[4]*256
                            self.Vx +=   databuf[5]
                            self.Yawz =  databuf[6]*256
                            self.Yawz += databuf[7]
                            self.Yawz = -self.Yawz
                            self.Vyaw =  databuf[8]*256
                            self.Vyaw += databuf[9]  
                            #self.Vyaw = -self.Vyaw                          
                            
                        else:
                            self.timer_odom.shutdown()
                            self.timer_communication.shutdown()
                            rospy.logerr("Invalid Index")
                            rospy.logerr()
                            pass
                else:
                    pass
            else:
                self.Circleloop.dequeue()
        else:
            # rospy.loginfo("Circle is Empty")
            pass   
    #Odom Timer call this to get velocity and imu info and convert to odom topic
    def timerOdomCB(self,event):
        #old version firmware have no version info and not support new command below
        outputdata = [0x5a,0x07,0x01,0x09,0x00,0x00,0x00]
        if self.LaserMode == "Express":
            outputdata[4] = 0x01
        elif self.LaserMode == "Boost":
            outputdata[4] = 0x02
        elif self.LaserMode == "Standard":
            outputdata[4] = 0x03
        else:
            outputdata[4] = 0x00    
        output = chr(0x5a) + chr(0x07) + chr(0x01) + chr(0x09) + chr(outputdata[4]) + chr(0x00) #0xdf is CRC-8 value
        crc_8 = self.crc_byte(outputdata,len(outputdata)-1)
        output += chr(crc_8)
        while(self.serialIDLE_flag):
            time.sleep(0.01)
        self.serialIDLE_flag = 1
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(output)
        except:
            rospy.logerr("Odom Command Send Faild")
        self.serialIDLE_flag = 0   
        #calculate odom data
        Vx = float(ctypes.c_int16(self.Vx).value/1000.0)
        Vy = float(ctypes.c_int16(self.Vy).value/1000.0)
        # rospy.loginfo('[base control]->Get car speed: Vx=%s',Vx)
        Vyaw = float(ctypes.c_int16(self.Vyaw).value/1000.0)
        Vyaw = (Vyaw/180)*3.14159265358979

        #self.pose_yaw = self.Yawz
        self.pose_yaw = float(ctypes.c_int16(self.Yawz).value/100.0)
        self.pose_yaw = self.pose_yaw*math.pi/180.0
        # rospy.loginfo('[base control]->Get pose_yaw = %s',self.pose_yaw)
        self.current_time = rospy.Time.now()
        dt = (self.current_time - self.previous_time).to_sec()
        self.previous_time = self.current_time
        self.pose_x = self.pose_x + Vx * self.kv * (math.cos(self.pose_yaw))*dt - Vy * self.kv * (math.sin(self.pose_yaw))*dt
        self.pose_y = self.pose_y + Vx * self.kv * (math.sin(self.pose_yaw))*dt + Vy * self.kv * (math.cos(self.pose_yaw))*dt
      
        #rospy.loginfo('@ # %s # %s # %s',self.pose_x,self.pose_y,self.pose_yaw)
        
        pose_quat = tf.transformations.quaternion_from_euler(0,0,self.pose_yaw)     
        orientation = Quaternion()
        
        
        msg = Odometry()
        imu_msg = Imu()
        msg.header.stamp = self.current_time
        msg.header.frame_id = self.odomId
        msg.child_frame_id =self.baseId
        msg.pose.pose.position.x = self.pose_x
        msg.pose.pose.position.y = self.pose_y
        msg.pose.pose.position.z = 0
        msg.pose.pose.orientation.x = pose_quat[0]
        msg.pose.pose.orientation.y = pose_quat[1]
        msg.pose.pose.orientation.z = pose_quat[2]
        msg.pose.pose.orientation.w = pose_quat[3]
        orientation.x,orientation .y,orientation .z,orientation .w=pose_quat[0],pose_quat[1],pose_quat[2],pose_quat[3]
        imu_msg.header.frame_id = self.imuId #self.imuId  self.baseId
        imu_msg.header.stamp = self.current_time
        
        msg.twist.twist.linear.x = Vx
        msg.twist.twist.linear.y = Vy
        msg.twist.twist.angular.z = Vyaw
        imu_msg.orientation= orientation 
        imu_msg.angular_velocity.z = Vyaw
        self.imu_pub.publish(imu_msg)
        if self.is_pub_odom_tf == 'true':
            self.tf_broadcaster.sendTransform((self.pose_x,self.pose_y,0.0),pose_quat,self.current_time,self.baseId,self.odomId)
        self.pub.publish(msg)

    #Battery Timer callback function to get battery info
    def timerBatteryCB(self,event):
        output = chr(0x5a) + chr(0x06) + chr(0x01) + chr(0x07) + chr(0x00) + chr(0xe4) #0xe4 is CRC-8 value
        while(self.serialIDLE_flag):
            time.sleep(0.01)
        self.serialIDLE_flag = 3
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(output)
        except:
            rospy.logerr("Battery Command Send Faild")
        self.serialIDLE_flag = 0
        msg = BatteryState()
        msg.header.stamp = self.current_time
        msg.header.frame_id = self.baseId
        msg.voltage = float(self.Vvoltage/1000.0)
        msg.current = float(self.Icurrent/1000.0)
        self.battery_pub.publish(msg)
        # rospy.loginfo('[base control]->Get battery data: Voltage=%s',msg.voltage)
        
        # CPU informatiom 
        CPU_temp = float(self.Raspi_info.getCPUtemperature())
        CPU_usage = float(self.Raspi_info.getCPUuse())
        
        # RAM information
        # Output is in kb, here I convert it in Mb for readability
        RAM_stats = self.Raspi_info.getRAMinfo()
        RAM_total = round(int(RAM_stats[0]) / 1000,1)
        RAM_used = round(int(RAM_stats[1]) / 1000,1)
        RAM_free = round(int(RAM_stats[2]) / 1000,1)
        RAM_perc = RAM_used*100/RAM_total
        
        # Disk information
        DISK_stats = self.Raspi_info.getDiskSpace()
        DISK_total = int((str(DISK_stats[0]))[0:-1])
        DISK_used = int((str(DISK_stats[1]))[0:-1])
        DISK_perc = int((str(DISK_stats[3]))[0:-1])
        
        ip_info = self.Raspi_info.getRaspiIPInfo()
        
        # rospy.loginfo('CPU Temperature = %.2f',CPU_temp)
        # rospy.loginfo('CPU usage = %.1f',CPU_usage)
        # rospy.loginfo('RAM total = %dMB',RAM_total)
        # rospy.loginfo('RAM used = %dMB',RAM_used)
        # rospy.loginfo('RAM perc = %.2f',RAM_perc)
        # rospy.loginfo('DISK total = %d GB',DISK_total)
        # rospy.loginfo('DISK used = %d GB',DISK_used)
        # rospy.loginfo('DISK perc = %d',DISK_perc)
        # rospy.loginfo('ip info : %s',ip_info)
        
        outputdata = [0x5a,0x1c,0x01,0x21,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
        outputdata[4] = (int(CPU_temp*100.0)>>8)&0xff
        outputdata[5] = int(CPU_temp*100.0)&0xff
        outputdata[6] = (int(CPU_usage*10.0)>>8)&0xff
        outputdata[7] = int(CPU_usage*10.0)&0xff
        outputdata[8] = (int(RAM_total)>>8)&0xff
        outputdata[9] = int(RAM_total)&0xff
        outputdata[10] = (int(RAM_used)>>8)&0xff
        outputdata[11] = int(RAM_used)&0xff
        outputdata[12] = (int(DISK_total)>>8)&0xff
        outputdata[13] = int(DISK_total)&0xff
        outputdata[14] = (int(DISK_used)>>8)&0xff
        outputdata[15] = int(DISK_used)&0xff
        outputdata[16] = 0
        if ip_info[0] == 'Master':
            outputdata[17] = 1
        elif ip_info[0] == 'Managed':
            outputdata[17] = 2
        else:
            outputdata[17] = 0
        outputdata[18] = int(ip_info[4])
        outputdata[19] = int(ip_info[3])
        outputdata[20] = int(ip_info[2])
        outputdata[21] = int(ip_info[1])
        outputdata[22] = int(ip_info[8])
        outputdata[23] = int(ip_info[7])
        outputdata[24] = int(ip_info[6])
        outputdata[25] = int(ip_info[5])
        output = chr(0x5a) + chr(0x1c) + chr(0x01) + chr(0x21) + \
            chr(outputdata[4]) + chr(outputdata[5]) + \
            chr(outputdata[6]) + chr(outputdata[7]) + \
            chr(outputdata[8]) + chr(outputdata[9]) + \
            chr(outputdata[10]) + chr(outputdata[11]) + \
            chr(outputdata[12]) + chr(outputdata[13]) + \
            chr(outputdata[14]) + chr(outputdata[15]) + \
            chr(outputdata[16]) + chr(outputdata[17]) + \
            chr(outputdata[18]) + chr(outputdata[19]) + \
            chr(outputdata[20]) + chr(outputdata[21]) + \
            chr(outputdata[22]) + chr(outputdata[23]) + \
            chr(outputdata[24]) + chr(outputdata[25]) + \
            chr(0x00)
        crc_8 = self.crc_byte(outputdata,len(outputdata)-1)
        output += chr(crc_8)
        while self.serialIDLE_flag:
            time.sleep(0.01)
        self.serialIDLE_flag = 4
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(output)
        except:
            rospy.logerr("Rpasi Info Command Send Faild")
        self.serialIDLE_flag = 0


#main function
if __name__=="__main__":
    try:
        rospy.init_node('base_control',anonymous=True)
        rospy.loginfo('base control start...')

        bc = BaseControl()
        rospy.spin()
    except KeyboardInterrupt:
        bc.serial.close
        print("Shutting down")
