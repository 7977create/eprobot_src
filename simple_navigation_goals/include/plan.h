#ifndef _PLAN_H_
#define _PLAN_H_

struct region
{
    float region_middle_x;
    float region_middle_y;

    float region_top;
    float region_botton;
    float region_left;
    float region_right;

    float region_angle_north;
    float region_angle_south;
    float region_angle_west;
    float region_angle_east;
};
struct region regions[11]={
    {0.3,8.24,9.04,7.68,-0.68,1.17,1.57,-1.57,3.14,0.0},     //1
    {4.56,8.01,8.72,7.38,3.53,5.57,1.57,-1.57,3.14,0.0},     //2
    {8.19,6.82,7.2,6.12,7.17,9.27,1.57,-2.141,1.57,-2.141},       //3 取到横向纵向最大值
    {12.4,7.69,8.51,6.96,11.6,13.5,1.57,-1.57,3.14,0.0},  //4
    {12.3,3.48,4.6,2.31,11.4,13.2,1.57,-1.57,3.14,0.0}, //5
    {12.1,0.225,0.778,-0.527,11.2,13.1,1.57,-1.57,3.14,0.0},//6
    {6.5,0.305,0.948,-0.381,5.36,7.61,1.57,-1.57,3.14,0.0},  //7
    {2.74,0.299,0.985,-0.451,1.7,3.85,1.57,-1.57,3.14,0.0},   //8
    {-0.281,0.454,0.984,-0.477,-1.15,0.718,1.57,-1.57,3.14,0.0},   //9
    {5.53,4.0,5.46,2.36,4.52,6.63,0.820,-2.151,1.973,-1.230}, //10 取到横向纵向最大值
    {7.27,5.68,6.87,4.57,6.33,8.56,0.854,-2.110,2.563,-0.481}, //11 取到横向纵向最大值
    };


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
float odom_position_x;
float odom_position_y;
float amcl_pose_x = 0.0;
float amcl_pose_y = 0.0;
geometry_msgs::Twist v;
geometry_msgs::PoseStamped this_pose_stamped;
ros::Publisher cancle_pub;
ros::Publisher vel_pub;
int order[20];
int i=0;
int num;
int set_flag = 0;
void angle_plan(void);
void angle_plan_a(void);
void angle_plan_b(void);
void angle_plan_c(void);

float a1;
float w1;
float z1;
float a2;
float w2;
float z2;
float pi = 3.141592653;
int order_1[120][14];//用来储存所有的排列组合顺序
int flag=0;
int rank_num;
float distance_sum[120];//用来储存所有组合的距离和
int temp[20];//用来储存需要排列的顺序
int temp_rank[120][14];
void swap(int &a,int &b);
void cal(int *a,int first,int length);
int times=0;
int a,b;
float min_dis;
int best;
int final_path[14];
int sum=0;
int count=0;
int sum_all=0;

unsigned int stat;
   // MoveBaseClient ac("move_base", true);
    //move_base_msgs::MoveBaseGoal goal;
    //goal.target_pose.header.frame_id = "map";
   // goal.target_pose.header.stamp = ros::Time::now();
   // angle_plan_a();
   // goal.target_pose.pose.position.x = regions[order[1]-1].region_middle_x;
   // goal.target_pose.pose.position.y = regions[order[1]-1].region_middle_y;
   // goal.target_pose.pose.orientation.w = w2;
   // goal.target_pose.pose.orientation.z = z2;
   // ac.sendGoal(goal);
    //ROS_INFO("%d ",order[1]);
#endif
