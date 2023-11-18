#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped
from gazebo_msgs.msg import LinkStates
from control_msgs.msg import JointControllerState
from std_msgs.msg import Header
from std_msgs.msg import Int32MultiArray
import numpy as np
import math
import tf2_ros
import time
import math
import string
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import ctypes
import tf
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped




class plan:
            def __init__(self):
                self.flag=0
                self.start_node=0
                self.end_node=0
                self.must_point=[]
                self.best_path=[]     
                self.all_path=[]  
                self.msg_sub=rospy.Subscriber('msg',Int32MultiArray, self.msg_callback, queue_size=1)
                self.msg_pub=rospy.Publisher('best_path', Int32MultiArray, queue_size=1)
                self.temp_end_node=0
                self.tmp=[]
                self.best_four=[[],[],[],[]]
            def plan_a(self):
                            path=[]
                            now_position=0
                            path_position=0
                            max_node_num=11
                            is_start_flag=[0]*(max_node_num+1)
                            final_path=[]
                            flag=0                           
                            link_array=np.array([
                                                [0,1,0,0,0,0,0,0,1,0,0,0],
                                                [1,0,1,1,0,0,0,0,0,1,0,0],
                                                [0,1,0,1,0,0,0,0,0,0,1,0],
                                                [0,1,1,0,1,0,0,0,0,0,0,0],
                                                [0,0,0,1,0,1,0,0,0,0,1,0],
                                                [0,0,0,0,1,0,1,0,0,0,0,0],
                                                [0,0,0,0,0,1,0,1,0,1,0,0],
                                                [0,0,0,0,0,0,1,0,1,1,0,0],
                                                [1,0,0,0,0,0,0,1,0,0,0,0],
                                                [0,1,0,0,0,0,1,1,0,0,1,0],
                                                [0,0,1,0,1,0,0,0,0,1,0,0],

                            ])
            
                            best_path=[]
                            distance=[]
                            path.append(self.start_node)
                            is_start_flag[self.start_node-1]=1
                            temp=0
                            path_position=path_position+1
                            while len(path) != 0 :
                                top_node = path[-1]
                                if top_node == self.end_node:
                                    final_path=path[:]
                                    if self.all_path.count(final_path)==1:
                                        break
                                    self.all_path.append(final_path)
                                    path_position=path_position-2
                                    if len(path)==2:
                                        now_position = path[-1]
                                        is_start_flag[path[-1] - 1] = 0;
                                        path.pop()
                                    elif len(path)==1:
                                        now_position = path[-1]
                                        is_start_flag[path[-1] - 1] = 0;
                                        for i in range(now_position,max_node_num+1):
                                            if link_array[top_node - 1][i] == 1:
                                                if len(path) >= 2:
                                                    if path[-2] == i + 1:
                                                        continue
                                                    #elif path[1] == self.end_node:
                                                        #continue
                                                   # elif path[-1] == 2 and path[-2] == 3 and i + 1 == 4:
                                                        #continue
                                                    #elif path[-1] == 4 and path[-2] == 3 and i + 1 == 2:
                                                        #continue
                                                    #elif path[-1] == 2 and path[-2] == 4 and i + 1 == 3:
                                                       # continue
                                                    #elif path[-1] == 4 and path[-2] == 2 and i + 1 == 3:
                                                        #continue
                                                    #elif path[-1] == 3 and path[-2] == 2 and i + 1 == 4:
                                                       # continue
                                                    #elif path[-1] == 2 and path[-2] == 3 and i + 1 == 4:
                                                        #continue
                                                    #elif path[-1] == 3 and path[-2] == 4 and i + 1 == 2:
                                                        #continue
                                                    else:
                                                        path_position = path_position + 1
                                                        #is_start_flag[i] = 1
                                                        path.append(i + 1)
                                                        now_position = 0
                                                        if i == 10:
                                                            i = 12
                                                        break
                                                else:
                                                    path_position = path_position + 1
                                                    #is_start_flag[i] = 1
                                                    path.append(i + 1)
                                                    now_position = 0
                                                    break

                                            if i == max_node_num:
                                                top_node = path[-1]
                                                #is_start_flag[top_node - 1] = 0
                                                now_position = path[-1]
                                            

                                    else:
                                        is_start_flag[path[-1] - 1] = 0
                                        path.pop()
                                        now_position = path[-1]
                                        is_start_flag[path[-1] - 1] = 0
                                        path.pop()

                                    if now_position==max_node_num:
                                        if path==[]:
                                            continue
                                        else:
                                                top_node=path[-1]
                                                is_start_flag[top_node-1]=0
                                                now_position=path[-1]
                                                path.pop()
                                                path_position=path_position-1
                                                #top_node=path[-1]

                                else:
                                    i=0
                                    for i in range(now_position,max_node_num+1):
                                        #if is_start_flag[i] == 0 and link_array[top_node-1][i] == 1:
                                        if path.count(path[-1])==2:
                                            flag=1
                                        if flag==1:
                                            if is_start_flag[i] == 0 and link_array[top_node - 1][i] == 1:
                                                if len(path) >= 2:
                                                    if path[-2] == i + 1:
                                                        continue
                                                    #elif path[1] == self.end_node:
                                                       # continue
                                                   # elif path[-1] == 2 and path[-2] == 3 and i + 1 == 4:
                                                       # continue
                                                   # elif path[-1] == 4 and path[-2] == 3 and i + 1 == 2:
                                                       # continue
                                                   # elif path[-1] == 2 and path[-2] == 4 and i + 1 == 3:
                                                      #  continue
                                                    #elif path[-1] == 4 and path[-2] == 2 and i + 1 == 3:
                                                       # continue
                                                    #elif path[-1] == 3 and path[-2] == 2 and i + 1 == 4:
                                                       # continue
                                                   # elif path[-1] == 2 and path[-2] == 3 and i + 1 == 4:
                                                        #continue
                                                    #elif path[-1] == 3 and path[-2] == 4 and i + 1 == 2:
                                                        #continue
                                                    else:
                                                        path_position = path_position + 1
                                                        is_start_flag[i] = 1
                                                        path.append(i + 1)
                                                        now_position = 0
                                                        if i == 10:
                                                            i = 12
                                                        break
                                                else:
                                                    path_position = path_position + 1
                                                    is_start_flag[i] = 1
                                                    path.append(i + 1)
                                                    now_position = 0
                                                    break

                                            if i == max_node_num:
                                                top_node = path[-1]
                                                is_start_flag[top_node - 1] = 0
                                                now_position = path[-1]
                                                path.pop()

                                        else:

                                            if link_array[top_node - 1][i] == 1:
                                                if len(path) >= 2:
                                                    if path[-2] == i + 1:
                                                        continue
                                                   # elif path[1] == self.end_node:
                                                        #continue
                                                   # elif path[-1] == 2 and path[-2] == 3 and i + 1 == 4:
                                                        #continue
                                                   # elif path[-1] == 4 and path[-2] == 3 and i + 1 == 2:
                                                        #continue
                                                    #elif path[-1] == 2 and path[-2] == 4 and i + 1 == 3:
                                                       # continue
                                                    #elif path[-1] == 4 and path[-2] == 2 and i + 1 == 3:
                                                        #continue
                                                    #elif path[-1] == 3 and path[-2] == 2 and i + 1 == 4:
                                                        #continue
                                                    #elif path[-1] == 2 and path[-2] == 3 and i + 1 == 4:
                                                        #continue
                                                    #elif path[-1] == 3 and path[-2] == 4 and i + 1 == 2:
                                                        #continue
                                                    else:
                                                        path_position = path_position + 1
                                                        #is_start_flag[i] = 1
                                                        path.append(i + 1)
                                                        now_position = 0
                                                        if i == 10:
                                                            i = 12
                                                        break
                                                else:
                                                    path_position = path_position + 1
                                                    #is_start_flag[i] = 1
                                                    path.append(i + 1)
                                                    now_position = 0
                                                    break

                                            if i == max_node_num:
                                                top_node = path[-1]
                                                #is_start_flag[top_node - 1] = 0
                                                now_position = path[-1]
                                                path.pop()
                          
            def final_plan(self):
                differ_point=[]
                best_path=[]
                best_all_path=[]
                distance=[]
                link_point=[[2,9],[1,3,4,10],[2,4,11],[2,3,5],[6,4,11],[7,5],[6,8,10],[7,9,10],[1,8],[2,11,7,8],[3,5,10]]
                #middle_point=[[1.09,-1.06],[5.79,-0.94],[10.1,-2.07],[15.5,-1.01],[15.6,-5.89],[15.6,-9.72],[8.41,-9.65],[4.29,-9.62],[1.07,-9.7],[7.19,-5.63],[9.14,-3.31]]
                middle_point=[[0.30,8.24],[4.56,8.01],[8.19,6.82],[12.4,7.69],[12.3,3.48],[12.1,0.225],[6.5,0.305],[2.74,0.299],[-0.281,0.454],[5.53,4.0],[7.27,5.68]]
                self.plan_a()

                for w in range(0,len(self.all_path)):
                        if set(self.must_point).issubset(set(self.all_path[w])):
                            self.tmp.append(w)
                #print(self.tmp)
                if len(self.tmp)==0:
                        for end_node in link_point[self.end_node-1]:
                            self.end_node=end_node
                            self.plan_a()
                                          

                        while [] in self.all_path:
                                    self.all_path.remove([])
                        for path_2 in self.all_path:
                            if path_2[-1]!=self.temp_end_node:
                                        path_2.append(self.temp_end_node)
                            
                        for i in range(0,len(self.all_path)):
                                if len(self.all_path[i])<=5:
                                        self.all_path[i]=[]
                                else:
                                        if self.all_path[i][-1]==self.all_path[i][-3]:
                                            self.all_path[i]=[]
                            
                        while [] in self.all_path:
                                    self.all_path.remove([])
                        for i in range(0,len(self.all_path)):
                            self.all_path[i].append([])              
                        for path_4 in self.all_path:
                            for i in range(0,len(path_4)-2):
                                if  path_4[i+1]==2 and path_4[i+2]==3 and path_4[i+3]==4:
                                    if path_4[i]==1 :
                                        path_4.insert(i+2,10)
                                        path_4.insert(i+3,11)
                                        #i=i+3
                                    elif path_4[i]==10 :
                                        path_4.insert(i+2,1)
                                        path_4.insert(i+3,9)
                                        path_4.insert(i+4,8)
                                        path_4.insert(i+5,10)
                                        path_4.insert(i+6,11)
                                        #i=i+6
                                
                                elif path_4[i]==4 and path_4[i+1]==3 and path_4[i+2]==2 :
                                    if path_4[i+3]==1:
                                        path_4.insert(i+2,11)
                                        path_4.insert(i+3,10) 
                                       # i=i+3
                                    elif  path_4[i+3]==10:
                                        path_4.insert(i+2,11)
                                        path_4.insert(i+3,10)
                                        path_4.insert(i+4,8)
                                        path_4.insert(i+5,9)
                                        path_4.insert(i+6,1)
                            #for i in range(0,len(path_4)-2):
                                elif path_4[i]==2 and path_4[i+1]==4 and path_4[i+2]==3:
                                    if path_4[i+3]==11:
                                        path_4.insert(i+1,10)
                                        path_4.insert(i+2,11)
                                        path_4.insert(i+3,5)
                                        #i=i+5
                                    else:
                                        path_4.insert(i+2,5)
                                        path_4.insert(i+3,11)
                                        
                                elif path_4[i]==3 and path_4[i+1]==2 and path_4[i+2]==4 :
                                        path_4.insert(i+2,10)
                                        path_4.insert(i+3,11)
                                        path_4.insert(i+4,3)
                                        #i=i+
                                elif path_4[i]==3 and path_4[i+1]==4 and path_4[i+2]==2 :

                                        path_4.insert(i+1,11)
                                        path_4.insert(i+2,5)
 
                                        #i=i+3
                                elif path_4[i]==4 and path_4[i+1]==2 and path_4[i+2]==3 :
                                    if path_4[i+3]==11:
                                        path_4.insert(i+1,5)
                                        path_4.insert(i+2,11)    
                                        path_4.insert(i+3,10)
                                        #path_4.insert(i+3,3)
                                    else:
                                        path_4.insert(i+2,10)
                                        path_4.insert(i+3,11)
                        
                        for i in range(0,len(self.all_path)):
                            self.all_path[i].pop()  



                        for path_5 in range(0,len(self.all_path)):
                            for i in range(0,len(self.all_path[path_5])-2):
                                if self.all_path[path_5][i]==self.all_path[path_5][i+2]:
                                    self.all_path[path_5]=[]
                                    break
                        while [] in self.all_path:
                             self.all_path.remove([])
                        for w in range(0,len(self.all_path)):
                                if set(self.must_point).issubset(set(self.all_path[w])):
                                    self.tmp.append(w)
                        for j in range(0,len(self.tmp)):
                            best_all_path.append(self.all_path[self.tmp[j]])
                        print(best_all_path)
                else:

                        for i in range(0,len(self.all_path)):
                            if len(self.all_path[i])<5:
                                self.all_path[i]=[]       

                        while [] in self.all_path:
                            self.all_path.remove([])

                        for i in range(0,len(self.all_path)):
                            self.all_path[i].append([])              
                        for path_4 in self.all_path:
                            for i in range(0,len(path_4)-2):
                                if  path_4[i+1]==2 and path_4[i+2]==3 and path_4[i+3]==4:
                                    if path_4[i]==1 :
                                        path_4.insert(i+2,10)
                                        path_4.insert(i+3,11)
                                        #i=i+3
                                    elif path_4[i]==10 :
                                        path_4.insert(i+2,1)
                                        path_4.insert(i+3,9)
                                        path_4.insert(i+4,8)
                                        path_4.insert(i+5,10)
                                        path_4.insert(i+6,11)
                                        #i=i+6
                                
                                elif path_4[i]==4 and path_4[i+1]==3 and path_4[i+2]==2 :
                                    if path_4[i+3]==1:
                                        path_4.insert(i+2,11)
                                        path_4.insert(i+3,10) 
                                       # i=i+3
                                    elif  path_4[i+3]==10:
                                        path_4.insert(i+2,11)
                                        path_4.insert(i+3,10)
                                        path_4.insert(i+4,8)
                                        path_4.insert(i+5,9)
                                        path_4.insert(i+6,1)
                            #for i in range(0,len(path_4)-2):
                                elif path_4[i]==2 and path_4[i+1]==4 and path_4[i+2]==3:
                                    if path_4[i+3]==11:
                                        path_4.insert(i+1,10)
                                        path_4.insert(i+2,11)
                                        path_4.insert(i+3,5)
                                        #i=i+5
                                    else:
                                        path_4.insert(i+2,5)
                                        path_4.insert(i+3,11)
                                        
                                elif path_4[i]==3 and path_4[i+1]==2 and path_4[i+2]==4 :
                                        path_4.insert(i+2,10)
                                        path_4.insert(i+3,11)
                                        path_4.insert(i+4,3)
                                        #i=i+
                                elif path_4[i]==3 and path_4[i+1]==4 and path_4[i+2]==2 :

                                        path_4.insert(i+1,11)
                                        path_4.insert(i+2,5)
 
                                        #i=i+3
                                elif path_4[i]==4 and path_4[i+1]==2 and path_4[i+2]==3 :
                                    if path_4[i+3]==11:
                                        path_4.insert(i+1,5)
                                        path_4.insert(i+2,11)    
                                        path_4.insert(i+3,10)
                                   
                                    else:
                                        path_4.insert(i+2,10)
                                        path_4.insert(i+3,11)
                        
                        for i in range(0,len(self.all_path)):
                            self.all_path[i].pop()  
                        print("bbbbbbbbbbbbbbbb")
                        print(self.all_path)
                        #print(self.all_path)
                        for path_6 in self.all_path:
                            if set(self.must_point).issubset(set(path_6)):
                                best_all_path.append(path_6)
                if best_all_path==[]:
                        self.all_path=[]
                        print("finallllllllllll")
                        self.end_node=self.start_node
                        self.start_node=self.temp_end_node  
                        self.temp_end_node=self.end_node
                        print(self.end_node)     
                        print(self.start_node) 
                        print(self.temp_end_node)        
                        self.plan_a()
                        
                        for w in range(0,len(self.all_path)):
                            if set(self.must_point).issubset(set(self.all_path[w])):
                                self.tmp.append(w)
                    
                        if(len(self.tmp)==0):
                                for end_node in link_point[self.end_node-1]:
                                    self.end_node=end_node
                                    self.plan_a()
                                            

                                while [] in self.all_path:
                                        self.all_path.remove([])
                                for path_2 in self.all_path:
                                    if path_2[-1]!=self.temp_end_node:
                                            path_2.append(self.temp_end_node)
                                
                                for i in range(0,len(self.all_path)):
                                        if len(self.all_path[i])<=5:
                                                self.all_path[i]=[]
                                        else:
                                                if self.all_path[i][-1]==self.all_path[i][-3]:
                                                    self.all_path[i]=[]
                                    
                                while [] in self.all_path:
                                            self.all_path.remove([])
                                for i in range(0,len(self.all_path)):
                                    self.all_path[i].append([])              
                                for path_4 in self.all_path:
                                    for i in range(0,len(path_4)-2):
                                        if  path_4[i+1]==2 and path_4[i+2]==3 and path_4[i+3]==4:
                                            if path_4[i]==1 :
                                                path_4.insert(i+2,10)
                                                path_4.insert(i+3,11)
                                                #i=i+3
                                            elif path_4[i]==10 :
                                                path_4.insert(i+2,1)
                                                path_4.insert(i+3,9)
                                                path_4.insert(i+4,8)
                                                path_4.insert(i+5,10)
                                                path_4.insert(i+6,11)
                                                #i=i+6
                                        
                                        elif path_4[i]==4 and path_4[i+1]==3 and path_4[i+2]==2 :
                                            if path_4[i+3]==1:
                                                path_4.insert(i+2,11)
                                                path_4.insert(i+3,10) 
                                            # i=i+3
                                            elif  path_4[i+3]==10:
                                                path_4.insert(i+2,11)
                                                path_4.insert(i+3,10)
                                                path_4.insert(i+4,8)
                                                path_4.insert(i+5,9)
                                                path_4.insert(i+6,1)
                                    #for i in range(0,len(path_4)-2):
                                        elif path_4[i]==2 and path_4[i+1]==4 and path_4[i+2]==3:
                                            if path_4[i+3]==11:
                                                path_4.insert(i+1,10)
                                                path_4.insert(i+2,11)
                                                path_4.insert(i+3,5)
                                                #i=i+5
                                            else:
                                                path_4.insert(i+2,5)
                                                path_4.insert(i+3,11)
                                                
                                        elif path_4[i]==3 and path_4[i+1]==2 and path_4[i+2]==4 :
                                                path_4.insert(i+2,10)
                                                path_4.insert(i+3,11)
                                                path_4.insert(i+4,3)
                                                #i=i+
                                        elif path_4[i]==3 and path_4[i+1]==4 and path_4[i+2]==2 :

                                                path_4.insert(i+1,11)
                                                path_4.insert(i+2,5)
        
                                                #i=i+3
                                        elif path_4[i]==4 and path_4[i+1]==2 and path_4[i+2]==3 :
                                            if path_4[i+3]==11:
                                                path_4.insert(i+1,5)
                                                path_4.insert(i+2,11)    
                                                path_4.insert(i+3,10)
                                                #path_4.insert(i+3,3)
                                            else:
                                                path_4.insert(i+2,10)
                                                path_4.insert(i+3,11)
                                
                                for i in range(0,len(self.all_path)):
                                    self.all_path[i].pop()  



                                for path_5 in range(0,len(self.all_path)):
                                    for i in range(0,len(self.all_path[path_5])-2):
                                        if self.all_path[path_5][i]==self.all_path[path_5][i+2]:
                                            self.all_path[path_5]=[]
                                            break
                                while [] in self.all_path:
                                    self.all_path.remove([])
                                
                                for w in range(0,len(self.all_path)):
                                        if set(self.must_point).issubset(set(self.all_path[w])):
                                            self.tmp.append(w)
                                for j in range(0,len(self.tmp)):
                                    best_all_path.append(self.all_path[self.tmp[j]])

                                for i in range(0,len(best_all_path)):
                                    best_all_path[i].reverse()

                                self.start_node=self.temp_end_node  
                                self.end_node=self.start_node
                                

                                
                        else:
                                for i in range(0,len(self.all_path)):
                                    if len(self.all_path[i])<=5:
                                        self.all_path[i]=[]       

                                while [] in self.all_path:
                                    self.all_path.remove([])
                                for i in range(0,len(self.all_path)):
                                    self.all_path[i].append([])              
                                for path_4 in self.all_path:
                                    for i in range(0,len(path_4)-2):
                                        if  path_4[i+1]==2 and path_4[i+2]==3 and path_4[i+3]==4:
                                            if path_4[i]==1 :
                                                path_4.insert(i+2,10)
                                                path_4.insert(i+3,11)
                                                #i=i+3
                                            elif path_4[i]==10 :
                                                path_4.insert(i+2,1)
                                                path_4.insert(i+3,9)
                                                path_4.insert(i+4,8)
                                                path_4.insert(i+5,10)
                                                path_4.insert(i+6,11)
                                                #i=i+6
                                        
                                        elif path_4[i]==4 and path_4[i+1]==3 and path_4[i+2]==2 :
                                            if path_4[i+3]==1:
                                                path_4.insert(i+2,11)
                                                path_4.insert(i+3,10) 
                                            # i=i+3
                                            elif  path_4[i+3]==10:
                                                path_4.insert(i+2,11)
                                                path_4.insert(i+3,10)
                                                path_4.insert(i+4,8)
                                                path_4.insert(i+5,9)
                                                path_4.insert(i+6,1)
                                    #for i in range(0,len(path_4)-2):
                                        elif path_4[i]==2 and path_4[i+1]==4 and path_4[i+2]==3:
                                            if path_4[i+3]==11:
                                                path_4.insert(i+1,10)
                                                path_4.insert(i+2,11)
                                                path_4.insert(i+3,5)
                                                #i=i+5
                                            else:
                                                path_4.insert(i+2,5)
                                                path_4.insert(i+3,11)
                                                
                                        elif path_4[i]==3 and path_4[i+1]==2 and path_4[i+2]==4 :
                                                path_4.insert(i+2,10)
                                                path_4.insert(i+3,11)
                                                path_4.insert(i+4,3)
                                                #i=i+
                                        elif path_4[i]==3 and path_4[i+1]==4 and path_4[i+2]==2 :

                                                path_4.insert(i+1,11)
                                                path_4.insert(i+2,5)
        
                                                #i=i+3
                                        elif path_4[i]==4 and path_4[i+1]==2 and path_4[i+2]==3 :
                                            if path_4[i+3]==11:
                                                path_4.insert(i+1,5)
                                                path_4.insert(i+2,11)    
                                                path_4.insert(i+3,10)
                                        
                                            else:
                                                path_4.insert(i+2,10)
                                                path_4.insert(i+3,11)
                                
                                for i in range(0,len(self.all_path)):
                                    self.all_path[i].pop()  

                                #print(self.all_path)
                                for path_6 in self.all_path:
                                    if set(self.must_point).issubset(set(path_6)):
                                        best_all_path.append(path_6)
                                for i in range(0,len(best_all_path)):
                                        best_all_path[i].reverse() 

                                self.start_node=self.temp_end_node  
                                self.end_node=self.start_node


                print(best_all_path)    
                print(self.end_node)
                print(self.start_node) 
                        #print(best_all_path)
                differ_point=[[],[],[],[]]
                short_dis=[[],[],[],[]]
                final_four=[[],[],[],[]]
                plan_a=[]
                plan_b=[]
                plan_c=[]
                plan_d=[]
                for r in range(0,len(best_all_path)):
                    temp=0
                    for a in range(0,len(best_all_path[r])-1):
                        temp=temp+(middle_point[best_all_path[r][a]-1][1]-middle_point[best_all_path[r][a+1]-1][1])*(middle_point[best_all_path[r][a]-1][1]-middle_point[best_all_path[r][a+1]-1][1])+(middle_point[best_all_path[r][a]-1][0]-middle_point[best_all_path[r][a+1]-1][0])*(middle_point[best_all_path[r][a]-1][0]-middle_point[best_all_path[r][a+1]-1][0])                                  
                    distance.append(temp)
            
                for i in range(0,len(link_point[self.start_node-1])):
                    for path in best_all_path:
                        if path[1]==link_point[self.start_node-1][i]:
                            #print(path)
                            differ_point[i].append(path)
                print(differ_point)
                plan_a=differ_point[0]
                plan_b=differ_point[1]
                plan_c=differ_point[2]
                plan_d=differ_point[3]
                
                if plan_a!=[]:
                    for r in range(0,len(plan_a)):
                        temp=0
                        for a in range(0,len(plan_a[r])-1):
                            temp=temp+((middle_point[plan_a[r][a]-1][1]-middle_point[plan_a[r][a+1]-1][1])*(middle_point[plan_a[r][a]-1][1]-middle_point[plan_a[r][a+1]-1][1])+(middle_point[plan_a[r][a]-1][0]-middle_point[plan_a[r][a+1]-1][0])*(middle_point[plan_a[r][a]-1][0]-middle_point[plan_a[r][a+1]-1][0]))**0.5                                 
                        short_dis[0].append(temp)
                if plan_b!=[]:
                    for r in range(0,len(plan_b)):
                        temp=0
                        for a in range(0,len(plan_b[r])-1):
                            temp=temp+((middle_point[plan_b[r][a]-1][1]-middle_point[plan_b[r][a+1]-1][1])*(middle_point[plan_b[r][a]-1][1]-middle_point[plan_b[r][a+1]-1][1])+(middle_point[plan_b[r][a]-1][0]-middle_point[plan_b[r][a+1]-1][0])*(middle_point[plan_b[r][a]-1][0]-middle_point[plan_b[r][a+1]-1][0]))**0.5                                  
                        short_dis[1].append(temp) 
                if plan_c!=[]:
                    for r in range(0,len(plan_c)):
                        temp=0
                        for a in range(0,len(plan_c[r])-1):
                            temp=temp+((middle_point[plan_c[r][a]-1][1]-middle_point[plan_c[r][a+1]-1][1])*(middle_point[plan_c[r][a]-1][1]-middle_point[plan_c[r][a+1]-1][1])+(middle_point[plan_c[r][a]-1][0]-middle_point[plan_c[r][a+1]-1][0])*(middle_point[plan_c[r][a]-1][0]-middle_point[plan_c[r][a+1]-1][0]))**0.5                                 
                        short_dis[2].append(temp) 
                if plan_d!=[]:
                    for r in range(0,len(plan_d)):
                        temp=0
                        for a in range(0,len(plan_d[r])-1):
                            temp=temp+((middle_point[plan_d[r][a]-1][1]-middle_point[plan_d[r][a+1]-1][1])*(middle_point[plan_d[r][a]-1][1]-middle_point[plan_d[r][a+1]-1][1])+(middle_point[plan_d[r][a]-1][0]-middle_point[plan_d[r][a+1]-1][0])*(middle_point[plan_d[r][a]-1][0]-middle_point[plan_d[r][a+1]-1][0]))**0.5                                 
                        short_dis[3].append(temp) 
                print(short_dis)

                for i in range(0,4):
                    if short_dis[i]!=[]:
                        best=short_dis[i].index(min(short_dis[i]))
                        self.best_four[i]=differ_point[i][best]
                print(self.best_four)
                


                #print(distance)
                best=distance.index(min(distance))
                best_path=best_all_path[best]
                self.best_path=best_path

                                #print(self.best_path)

            
            def msg_callback(self,msg):
                print "get msg len=%d, msg data:" %len(msg.data)
                print(msg.data)
                if self.flag==0:
                    self.must_point=[0]*(len(msg.data)-2)
                    self.start_node=msg.data[0]
                    for i in range(0,len(msg.data)-2):
                        self.must_point[i]=(msg.data[i+1])
                    self.end_node  = msg.data[len(msg.data)-1]
                    self.temp_end_node=msg.data[len(msg.data)-1]
                    self.final_plan()
                    self.flag=1
                else:
                    pub_msg=Int32MultiArray()
                    print "final paths:"
                    for i in range(0,4):
                        pub_msg.data.append(len(self.best_four[i]))
                        print(self.best_four[i])
                        for j in range(0,len(self.best_four[i])):
                            pub_msg.data.append(self.best_four[i][j])
                    #for i in range(0,len(self.best_path)):
                       # pub_msg.data.append(self.best_path[i])
                    # add num to first of buf
                    #pub_msg.data.insert(0,len(pub_msg.data))
                    print(pub_msg.data)
                    self.msg_pub.publish(pub_msg)
                    self.flag=0
                    
if __name__ == "__main__":
    rospy.init_node("plan")
    obj=plan()
    rospy.spin()
    
