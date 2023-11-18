

import sys
import math



pi = 3.141592653
list_point = [[0] * 2] * 12
# need to change the center axis values of points
list_point[0] = [1,-0.75] # 1
list_point[1] = [6,-0.75] # 2
list_point[2] = [10.6,-2] # 3
list_point[3] = [16,-0.75] # 4
list_point[4] = [16,-5.85] # 5
list_point[5] = [16,-9.75] # 6
list_point[6] = [9,-9.75] # 7
list_point[7] = [4.5,-9.75] # 8
list_point[8] = [1,-9.75] # 9
list_point[9] = [7.56,-5.4] # 10
list_point[10] = [9.53,-3.2] # 11



list_target = [2,10,11,3,4,5] # result of plan
num_target = 6 # number of plan
list_orientation = [([0] * 2) for i in range(num_target)] # result of targets orientation[w,z]

def get_target_orientation(list_in,num):
    list_temp = [([0] * 2) for i in range(num)]
    for i in range(0,num-1):
        x0 = list_point[list_in[i]-1][0]
        x1 = list_point[list_in[i+1]-1][0]
        y0 = list_point[list_in[i]-1][1]
        y1 = list_point[list_in[i+1]-1][1]
        dx = x1-x0
        dy = y1-y0
        if dx == 0 and dy > 0:
            a = pi/2
        elif dx == 0 and dy < 0:
            a = -pi/2
        elif dx>0:
            a = math.atan(dy/dx)
        elif dx<0 and dy>0:
            a = math.atan(dy/dx) + pi
        elif dx<0 and dy<0:
            a = math.atan(dy/dx) - pi
        elif dy == 0 and dx>0:
            a = 0
        elif dy == 0 and dx<0:
            a = pi
        list_temp[i][0] = math.cos(a/2)
        list_temp[i][1] = math.sin(a/2)   
    list_temp[num-1][0] = list_temp[num-2][0]
    list_temp[num-1][1] = list_temp[num-2][1]
    
    return list_temp
    


if True :
    list_orientation = get_target_orientation(list_target,num_target)
    print list_orientation