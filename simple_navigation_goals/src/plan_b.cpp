#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <actionlib_msgs/GoalID.h>
#include "plan.h"
#include <iostream>
using namespace std;

ros::Publisher msg_pub;
void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclPose)//ConstPtr
 {
    //geometry_msgs::PoseStamped this_pose_stamped;
    amcl_pose_x = amclPose->pose.pose.position.x;
    amcl_pose_y = amclPose->pose.pose.position.y;

 }
void msgCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    //接收规划器发布的最佳路径顺序
    int sum_1;
    string en;
    sum_1 = msg->data.at(0);
    sum = msg->data.at(sum_1+1);
    if(order[0]!=0)
        flag=1;
    ROS_INFO("get plan!Steps: %d",sum);
    for(int i=0;i<sum;i++)
    {
        order[i]=msg->data.at(sum_1+2+i);
        ROS_INFO("Step No%d: point %d",i+1,order[i]);
    }
    ROS_INFO("Please Confirm the Path: y/n");
    std::cin>>en;
    if(en=="n")
        ros::shutdown();
    
   
}
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom)//ConstPtr
 {
    float mid_top,mid_bot,mid_left,mid_right;
    float temp_top,temp_bot,temp_left,temp_right;
    //获取自己的实时位置信息
    this_pose_stamped.pose.position.x = amcl_pose_x;
    this_pose_stamped.pose.position.y = amcl_pose_y;
    if(flag==0)//还未发送途径点并收到最佳路径
    {
        for(int q=0;q<11;q++)//获取自身所在的区域
        {
            if((this_pose_stamped.pose.position.x>=regions[q].region_left&&this_pose_stamped.pose.position.x<=regions[q].region_right&&this_pose_stamped.pose.position.y>=regions[q].region_botton&&this_pose_stamped.pose.position.y<=regions[q].region_top))
            {
                    //将起点和终点放到数组的首尾                  
                    temp[0]=q+1;
                    temp[num-1]=order[0];
                    ROS_INFO("start:%d,end:%d",temp[0],temp[num-1]);
                    break;//获取后就跳出
            }
            
        }
      std_msgs::Int32MultiArray msg;
       
       for(int i=0;i<num;i++)
       {
        msg.data.push_back(temp[i]);
       }
     msg_pub.publish(msg);//发布起点终点和途径点给规划器
    }
    if(flag==1)
    {
        if(i==0)
        {
            mid_top=regions[order[i]-1].region_top;
            mid_bot=regions[order[i]-1].region_botton;
            mid_left=regions[order[i]-1].region_left;
            mid_right=regions[order[i]-1].region_right;
        }
        else
        {
            //1/2 
            temp_top=(regions[order[i-1]-1].region_top+regions[order[i]-1].region_top)/2;
            temp_bot=(regions[order[i-1]-1].region_botton+regions[order[i]-1].region_botton)/2;
            temp_left=(regions[order[i-1]-1].region_left+regions[order[i]-1].region_left)/2;
            temp_right=(regions[order[i-1]-1].region_right+regions[order[i]-1].region_right)/2;
            //3/4
            mid_top=(temp_top+regions[order[i]-1].region_top)/2;
            mid_bot=(temp_bot+regions[order[i]-1].region_botton)/2;
            mid_left=(temp_left+regions[order[i]-1].region_left)/2;
            mid_right=(temp_right+regions[order[i]-1].region_right)/2;
            //5/8
            // mid_top=(mid_top+temp_top)/2;
            // mid_bot=(mid_bot+temp_top)/2;
            // mid_left=(mid_left+temp_top)/2;
            // mid_right=(mid_right+temp_top)/2;
            
            
        }
        
        if((this_pose_stamped.pose.position.x>=mid_left&&this_pose_stamped.pose.position.x<=mid_right&&this_pose_stamped.pose.position.y>=mid_bot&&this_pose_stamped.pose.position.y<=mid_top))
        {
            if(set_flag == 0)
                set_flag = 1;
            //ROS_INFO("reach now!");
            MoveBaseClient ac("move_base", true);
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            angle_plan_b();
            goal.target_pose.pose.position.x = regions[order[i+1]-1].region_middle_x;
            goal.target_pose.pose.position.y = regions[order[i+1]-1].region_middle_y;
            goal.target_pose.pose.orientation.w = w2;
            goal.target_pose.pose.orientation.z = z2;
            //ROS_INFO("Sending goal");
            if(set_flag==1)
            {
                ROS_INFO("Sending goal:No%d",i+1);
                goal.target_pose.pose.position.x+0.3;
                goal.target_pose.pose.position.y+0.3;
                ac.sendGoal(goal);
                ros::Duration(0.3).sleep();
                ROS_INFO("first Sending goal");
                goal.target_pose.pose.position.x-0.3;
                goal.target_pose.pose.position.y+0.3;
                ac.sendGoal(goal);
                ros::Duration(0.3).sleep();
                ROS_INFO("second Sending goal");
                goal.target_pose.pose.position.x+0.3;
                goal.target_pose.pose.position.y-0.3;
                ac.sendGoal(goal);
                ros::Duration(0.3).sleep();
                ROS_INFO("third Sending goal");
                goal.target_pose.pose.position.x-0.3;
                goal.target_pose.pose.position.y-0.3;
                ac.sendGoal(goal);
                ros::Duration(0.3).sleep();
                ROS_INFO("forth Sending goal");
                goal.target_pose.pose.position.x;
                goal.target_pose.pose.position.y;
                ac.sendGoal(goal);
                ROS_INFO("fifth Sending goal");
                set_flag = 0;
                i=i+1;
            }
        
            
            ROS_INFO("next goal:No%d",i+1);

            if(i==sum-1)
            {
                ROS_INFO("stop!");
                actionlib_msgs::GoalID goals;
                cancle_pub.publish(goals);
                ros::Duration(3.0).sleep();
                ros::shutdown();
            }
        }
        else
        {
            set_flag = 0;
        }
    }
 }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n;
  MoveBaseClient ac("move_base", true);
  num=argc;
   for(int j=0;j<num-1;j++)
  {
      if(strcmp(argv[j+1],"1")==0)order[j]=1;
      else if(strcmp(argv[j+1],"2")==0)order[j]=2;
      else if(strcmp(argv[j+1],"3")==0)order[j]=3;
      else if(strcmp(argv[j+1],"4")==0)order[j]=4;
      else if(strcmp(argv[j+1],"5")==0)order[j]=5;
      else if(strcmp(argv[j+1],"6")==0)order[j]=6;
      else if(strcmp(argv[j+1],"7")==0)order[j]=7;
      else if(strcmp(argv[j+1],"8")==0)order[j]=8;
      else if(strcmp(argv[j+1],"9")==0)order[j]=9;
      else if(strcmp(argv[j+1],"10")==0)order[j]=10;
      else if(strcmp(argv[j+1],"11")==0)order[j]=11;
  }
  ROS_INFO("end point: %d",order[0]);
  for(int r=0;r<num-2;r++)
  {
      temp[r+1]=order[r+1];
  } 
  while(!ac.waitForServer(ros::Duration(1.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  angle_plan();
  ros::Subscriber odom_sub = n.subscribe("/odometry/filtered",10,odomCallback);
  ros::Subscriber amcl_sub = n.subscribe("/amcl_pose",10,amclCallback);
  ros::Subscriber msg_sub = n.subscribe("/best_path",10,msgCallback);
  ros::Publisher cancle_pub = n.advertise<actionlib_msgs::GoalID>("move_base/cancel",1);
  msg_pub    = n.advertise<std_msgs::Int32MultiArray>("msg", 1000);

  ros::spin();
  return 0;
}





void angle_plan(void)
{
     if(regions[order[2]-1].region_middle_x==regions[order[1]-1].region_middle_x&&regions[order[2]-1].region_middle_y>regions[order[1]-1].region_middle_y)
  {
      a1=regions[order[1]-1].region_angle_south;
  }
  else if(regions[order[2]-1].region_middle_x==regions[order[1]-1].region_middle_x&&regions[order[2]-1].region_middle_y<regions[order[1]-1].region_middle_y)
  {
      a1=regions[order[1]-1].region_angle_north;
  }
   else if(regions[order[2]-1].region_middle_x>regions[order[1]-1].region_middle_x&&regions[order[2]-1].region_middle_y==regions[order[1]-1].region_middle_y)
  {
     a1=regions[order[1]-1].region_angle_east;
  }
  else if(regions[order[2]-1].region_middle_x<regions[order[1]-1].region_middle_x&&regions[order[2]-1].region_middle_y==regions[order[1]-1].region_middle_y)
  {
     a1=regions[order[1]-1].region_angle_west;
  }
  else if(regions[order[2]-1].region_middle_x>regions[order[1]-1].region_middle_x&&regions[order[2]-1].region_middle_y>regions[order[1]-1].region_middle_y)
  {
      a1=regions[order[1]-1].region_angle_north;
  }
  else if(regions[order[2]-1].region_middle_x>regions[order[1]-1].region_middle_x&&regions[order[2]-1].region_middle_y<regions[order[1]-1].region_middle_y)
  {
      a1=regions[order[1]-1].region_angle_east;
  }
   else if(regions[order[2]-1].region_middle_x<regions[order[1]-1].region_middle_x&&regions[order[2]-1].region_middle_y>regions[order[1]-1].region_middle_y)
  {
      a1=regions[order[1]-1].region_angle_west;
  }
   else if(regions[order[2]-1].region_middle_x<regions[order[1]-1].region_middle_x&&regions[order[2]-1].region_middle_y<regions[order[1]-1].region_middle_y)
  {
      a1=regions[order[1]-1].region_angle_south;
  }
  w1=cos(a1/2);
  z1=sin(a1/2);
}
void angle_plan_a(void)
{
     if(regions[order[i+3]-1].region_middle_x==regions[order[i+2]-1].region_middle_x&&regions[order[i+3]-1].region_middle_y>regions[order[i+2]-1].region_middle_y)
  {
      a2=regions[order[i+2]-1].region_angle_south;
  }
  else if(regions[order[i+3]-1].region_middle_x==regions[order[i+2]-1].region_middle_x&&regions[order[i+3]-1].region_middle_y<regions[order[i+2]-1].region_middle_y)
  {
      a2=regions[order[i+2]-1].region_angle_north;
  }
   else if(regions[order[i+3]-1].region_middle_x>regions[order[i+2]-1].region_middle_x&&regions[order[i+3]-1].region_middle_y==regions[order[i+2]-1].region_middle_y)
  {
     a2=regions[order[i+2]-1].region_angle_east;
  }
  else if(regions[order[i+3]-1].region_middle_x<regions[order[i+2]-1].region_middle_x&&regions[order[i+3]-1].region_middle_y==regions[order[i+2]-1].region_middle_y)
  {
     a2=regions[order[i+2]-1].region_angle_west;
  }
  else if(regions[order[i+3]-1].region_middle_x>regions[order[i+2]-1].region_middle_x&&regions[order[i+3]-1].region_middle_y>regions[order[i+2]-1].region_middle_y)
  {
      a2=regions[order[i+2]-1].region_angle_north;
  }
  else if(regions[order[i+3]-1].region_middle_x>regions[order[i+2]-1].region_middle_x&&regions[order[i+3]-1].region_middle_y<regions[order[i+2]-1].region_middle_y)
  {
      a2=regions[order[i+2]-1].region_angle_east;
  }
   else if(regions[order[i+3]-1].region_middle_x<regions[order[i+2]-1].region_middle_x&&regions[order[i+3]-1].region_middle_y>regions[order[i+2]-1].region_middle_y)
  {
      a2=regions[order[i+2]-1].region_angle_west;
  }
   else if(regions[order[i+3]-1].region_middle_x<regions[order[i+2]-1].region_middle_x&&regions[order[i+3]-1].region_middle_y<regions[order[i+2]-1].region_middle_y)
  {
      a2=regions[order[1]-1].region_angle_south;
  }
  w2=cos(a2/2);
  z2=sin(a2/2);
}

void angle_plan_b(void)
{
    float x0,y0,x1,y1,dx,dy;

    if(i==sum-2)
    {
        x0 = regions[order[i]-1].region_middle_x;
        x1 = regions[order[i+1]-1].region_middle_x;
        y0 = regions[order[i]-1].region_middle_y;
        y1 = regions[order[i+1]-1].region_middle_y;
    }
    else
    {
        x0 = regions[order[i+1]-1].region_middle_x;
        x1 = regions[order[i+2]-1].region_middle_x;
        y0 = regions[order[i+1]-1].region_middle_y;
        y1 = regions[order[i+2]-1].region_middle_y;
    }   
    dx = x1-x0;
    dy = y1-y0;
    if(dx == 0 && dy > 0) 
        a2 = pi/2;
    else if(dx == 0 && dy < 0) 
        a2 = -pi/2;
    else if(dx>0) 
        a2 = atan(dy/dx);
    else if(dx<0 && dy>0) 
        a2 = atan(dy/dx) + pi;
    else if(dx<0 && dy<0) 
        a2 = atan(dy/dx) - pi;
    else if(dy == 0 && dx>0) 
        a2 = 0;
    else if(dy == 0 && dx<0) 
        a2 = pi;
    w2=cos(a2/2);
    z2=sin(a2/2);
    ROS_INFO("point: %d, w=%f,z=%f",order[i+1],w2,z2);
}

void angle_plan_c(void)
{
    float x0,y0,x1,y1,dx,dy;

    x0 = regions[order[i]-1].region_middle_x;
    x1 = regions[order[i+1]-1].region_middle_x;
    y0 = regions[order[i]-1].region_middle_y;
    y1 = regions[order[i+1]-1].region_middle_y;
    
    dx = x1-x0;
    dy = y1-y0;
    if(dx == 0 && dy > 0) 
        a2 = pi/2;
    else if(dx == 0 && dy < 0) 
        a2 = -pi/2;
    else if(dx>0) 
        a2 = atan(dy/dx);
    else if(dx<0 && dy>0) 
        a2 = atan(dy/dx) + pi;
    else if(dx<0 && dy<0) 
        a2 = atan(dy/dx) - pi;
    else if(dy == 0 && dx>0) 
        a2 = 0;
    else if(dy == 0 && dx<0) 
        a2 = pi;
    w2=cos(a2/2);
    z2=sin(a2/2);
    ROS_INFO("point: %d, w=%f,z=%f",order[i+1],w2,z2);
}


void swap(int &a,int &b)
{
     int tmp;
     tmp = a;
     a = b;
     b = tmp;
}
void cal(int *a,int first,int length){
     if(first == length){
          for(int i = 0; i <= length; i++)
          {
               temp_rank[times][i]=a[i];
          }
          times=times+1;
     }
     else{
          for(int i = first; i <= length; i++){
               //循环遍历使得当前位置后边的每一个元素都和当前深度的第一个元素交换一次
               swap(a[first],a[i]);//使得与第一个元素交换
               cal(a,first+1,length);//深入递归，此时已确定前边的元素，处理后边子序列的全排列形式。
               swap(a[first],a[i]);//恢复交换之前的状态
         }
          
     }
}



