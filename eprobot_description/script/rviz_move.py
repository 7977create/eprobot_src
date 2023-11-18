#!/usr/bin/env python

import rospy, math
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
import os

def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
  if omega == 0 or v == 0:
    return 0

  radius = v / omega
  return math.atan(wheelbase / radius)

def cal_deltaV(v, omega, wheelbase):
  if omega == 0 or v == 0:
    return 0

  return v * 0.2 * math.tan(omega) / (2 * wheelbase)

def cmd_callback(data):
  global init_pose, vel, steer, dv
  v = data.linear.x
  steering = convert_trans_rot_vel_to_steering_angle(v, data.angular.z, wheelbase)
  jerk = cal_deltaV(v, data.angular.z, wheelbase)
  vel = v * 2.0 / 0.065 
  steer = steering
  dv = jerk * 2.0 / 0.065 



def talker(base,v,dv,steer):
    if base > 3.141592658 :
        base -= 3.1415926575
    elif base < -3.141592658 :
        base += 3.1415926575

    if steer > 0.5 :
        steer = 0.5
    elif steer < -0.5 :
        steer = -0.5

    pub = rospy.Publisher('joint_states', JointState, queue_size=10)

    rate = rospy.Rate(10) # 10hz
    pub_joint = JointState()
    pub_joint.header = Header()
    pub_joint.header.stamp = rospy.Time.now()
    pub_joint.name = ['left_steer_joint', 'left_front_wheel_joint', 'right_steer_joint', 'right_front_wheel_joint','right_rear_wheel_joint', 'left_rear_wheel_joint']
    pub_joint.position = [steer,base+v,steer,-base-v,-base-v,base+v]
    pub_joint.velocity = []
    pub_joint.effort = []
    pub.publish(pub_joint)
    rate.sleep()
    return base+v

if __name__ == '__main__':

    init_pose = 0.0
    vel = 0.0
    steer = 0.0
    dv = 0.0

    f = os.popen(r"rosnode list","r")
    d = f.read()
    #print(d)
    f.close()
    if d.find("/joint_state_publisher_gui") != -1 :
        f = os.popen(r"rosnode kill /joint_state_publisher_gui","r")
        d = f.read()
        f.close()
    
    rospy.init_node('eprobot_joint_publisher')
    
    twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel') 
    wheelbase = rospy.get_param('~wheelbase', 0.18)

    rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=5)
    rospy.loginfo("Node 'rviz_move' started.\n")

    while not rospy.is_shutdown():
        try:
            init_pose = talker(init_pose, vel, dv, steer)
        except rospy.ROSInterruptException:
            pass



