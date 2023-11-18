import numpy as np
start_node=1
end_node=2
path=[]
all_path=[]
now_position=0
path_position=0
max_node_num=11
is_start_flag=[0]*(max_node_num+1)
final_path=[]
flag=0
must_point=[10,9,6]
tmp=[]
#                    1,2,3,4,5,6,7,8,9,10,11
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
path.append(start_node)
is_start_flag[start_node-1]=1
temp=0
path_position=path_position+1
while len(path) != 0 :
    top_node = path[-1]
    if top_node == end_node:
        final_path=path[:]
        all_path.append(final_path)
        path_position=path_position-2
        if len(path)<=2:
            now_position = path[-1]
            is_start_flag[path[-1] - 1] = 0;
            path.pop()

        else:
            is_start_flag[path[-1] - 1] = 0;
            path.pop()
            now_position = path[-1]
            is_start_flag[path[-1] - 1] = 0
            path.pop()

        if now_position==max_node_num:
                    top_node=path[-1]
                    is_start_flag[top_node-1]=0
                    now_position=path[-1]
                    path.pop()
                    path_position=path_position-1
                    top_node=path[-1]

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
                        elif path[1] == end_node:
                            continue
                        elif path[-1] == 2 and path[-2] == 3 and i + 1 == 4:
                            continue
                        elif path[-1] == 4 and path[-2] == 3 and i + 1 == 2:
                            continue
                        elif path[-1] == 2 and path[-2] == 4 and i + 1 == 3:
                            continue
                        elif path[-1] == 4 and path[-2] == 2 and i + 1 == 3:
                            continue
                        elif path[-1] == 3 and path[-2] == 2 and i + 1 == 4:
                            continue
                        elif path[-1] == 2 and path[-2] == 3 and i + 1 == 4:
                            continue
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
                        elif path[1] == end_node:
                            continue
                        elif path[-1] == 2 and path[-2] == 3 and i + 1 == 4:
                            continue
                        elif path[-1] == 4 and path[-2] == 3 and i + 1 == 2:
                            continue
                        elif path[-1] == 2 and path[-2] == 4 and i + 1 == 3:
                            continue
                        elif path[-1] == 4 and path[-2] == 2 and i + 1 == 3:
                            continue
                        elif path[-1] == 3 and path[-2] == 2 and i + 1 == 4:
                            continue
                        elif path[-1] == 2 and path[-2] == 3 and i + 1 == 4:
                            continue
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




for w in range(0,len(all_path)):
    if set(must_point).issubset(set(all_path[w])):
        tmp.append(w)

for r in range(0,len(tmp)):
    print(all_path[tmp[r]])












