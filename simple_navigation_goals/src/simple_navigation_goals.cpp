#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
 
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <actionlib_msgs/GoalID.h>



typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

float odom_position_x;
float odom_position_y;
float amcl_pose_x = 0.0;
float amcl_pose_y = 0.0;
geometry_msgs::Twist v;
geometry_msgs::PoseStamped this_pose_stamped;
ros::Publisher cancle_pub;
ros::Publisher vel_pub;


void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclPose)//ConstPtr
 {
    //geometry_msgs::PoseStamped this_pose_stamped;
    amcl_pose_x = amclPose->pose.pose.position.x;
    amcl_pose_y = amclPose->pose.pose.position.y;

 }



void odomCallback(const nav_msgs::Odometry::ConstPtr& odom)//ConstPtr
 {
   
    //geometry_msgs::PoseStamped this_pose_stamped;
    //this_pose_stamped.pose.position.x = odom->pose.pose.position.x;
    this_pose_stamped.pose.position.x = amcl_pose_x;
    this_pose_stamped.pose.position.y = amcl_pose_y;

    if((this_pose_stamped.pose.position.x>=34.794&&this_pose_stamped.pose.position.x<=36.606&&this_pose_stamped.pose.position.y>=-0.036&&this_pose_stamped.pose.position.y<=1.500))
    {
    MoveBaseClient ac("move_base", true);
    move_base_msgs::MoveBaseGoal goal;
    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
     float a2=3.14;
    float w2=cos(a2/2);
     float z2=sin(a2/2);
    goal.target_pose.pose.position.x = -1.143;//-1.143
    goal.target_pose.pose.position.y = 1.645;
    goal.target_pose.pose.orientation.w = w2;
    goal.target_pose.pose.orientation.z = z2;
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
    }

 }

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n;

  MoveBaseClient ac("move_base", true);
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(1.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
   move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  float a1=2.50;
  float w1=cos(a1/2);
  float z1=sin(a1/2);
  goal.target_pose.pose.position.x = 35.231;
  goal.target_pose.pose.position.y = 1.317;//1.017
  goal.target_pose.pose.orientation.z = z1;
  goal.target_pose.pose.orientation.w = w1;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  ros::Subscriber odom_sub = n.subscribe("/odometry/filtered",10,odomCallback);
  ros::Subscriber amcl_sub = n.subscribe("/amcl_pose",10,amclCallback);
  ros::Publisher cancle_pub = n.advertise<actionlib_msgs::GoalID>("move_base/cancel",1);
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  
  

  ros::spin();
  
  return 0;
}
