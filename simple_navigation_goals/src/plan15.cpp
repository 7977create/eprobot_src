/*
Copyright (c) 2017, ChanYuan KUO, YoRu LU,
latest editor: HaoChih, LIN
All rights reserved. (Hypha ROS Workshop)

This file is part of hypha_racecar package.

hypha_racecar is free software: you can redistribute it and/or modify
it under the terms of the GNU LESSER GENERAL PUBLIC LICENSE as published
by the Free Software Foundation, either version 3 of the License, or
any later version.

hypha_racecar is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU LESSER GENERAL PUBLIC LICENSE for more details.

You should have received a copy of the GNU LESSER GENERAL PUBLIC LICENSE
along with hypha_racecar.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <stdlib.h>
#include "ros/ros.h"
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "nav_msgs/Path.h"
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#define PI 3.14159265358979
#define num 80
/********************/
/* CLASS DEFINITION */
/********************/
class L1Controller
{
public:
    L1Controller();
    void initMarker();
    bool isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose);
    bool isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos);
    double getYawFromPose(const geometry_msgs::Pose& carPose);
    double getEta(const geometry_msgs::Pose& carPose);
    double getCar2GoalDist();
    double getL1Distance(const double& _Vcmd);
    double getSteeringAngle(double eta);
    double getGasInput(const float& current_v);
    double curvature(Eigen::Matrix<double, 3, 1>& x, Eigen::Matrix<double, 3, 1>& y);
    void sort(double* a, int length, int* b);
    double transfrom_anglur(double anglur);
    geometry_msgs::Point get_odom_car2WayPtVec(const geometry_msgs::Pose& carPose);

private:
    ros::NodeHandle n_;
    ros::Subscriber odom_sub, path_sub, goal_sub;
    ros::Publisher pub_, marker_pub, three_point_pub;
    ros::Timer timer1, timer2, timer3;
    tf::TransformListener tf_listener, listener;
    Eigen::Matrix<double, 3, 1> path_near_x;
    Eigen::Matrix<double, 3, 1> path_near_y;
    Eigen::Matrix<double, 3, 1> path_far_x;
    Eigen::Matrix<double, 3, 1> path_far_y;
    double path_near_kappa, path_far_kappa;
    visualization_msgs::Marker points, line_strip, goal_circle;
    int index, i, j;
    visualization_msgs::Marker three_points;
    geometry_msgs::Twist cmd_vel;
    geometry_msgs::Point odom_goal_pos;
    nav_msgs::Odometry odom;
    nav_msgs::Path map_path, odom_path;
    double L, Lfw, Lrv, Vcmd, lfw, lrv, steering, u, v, prospect, k_far;
    double Gas_gain, baseAngle, Angle_gain, goalRadius;
    double baseSpeed, odom_x, odom_y, follow_x, follow_y;
    double e, yaw, ch, delta_e, theta, min, car_x, car_y, car_yaw;
    int controller_freq, b[num];
    bool foundForwardPt, goal_received, goal_reached;
    double min_distance[num];
    double look_ahead = 0.6;
    double k_stanley = 0.4;
    double dx, dy;
    double alpha;
    double last_delta, now_dalta;
    double k_soft = 0;
    void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
    void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
    void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
    void goalReachingCB(const ros::TimerEvent&);
    void controlLoopCB(const ros::TimerEvent&);
    void getodomCB(const ros::TimerEvent&);

}; // end of class


L1Controller::L1Controller()
{
    //Private parameters handler
    ros::NodeHandle pn("~");

    //Car parameter
    pn.param("L", L, 0.26);
    pn.param("Lrv", Lrv, 10.0);
    pn.param("Vcmd", Vcmd, 1.0);
    pn.param("lfw", lfw, 0.13);
    pn.param("lrv", lrv, 10.0);
    pn.param("prospect", prospect, 40.0);
    //Controller parameter
    pn.param("controller_freq", controller_freq, 20);
    pn.param("AngleGain", Angle_gain, 1.0);
    pn.param("GasGain", Gas_gain, 1.0);
    pn.param("baseSpeed", baseSpeed, 2.5);
    pn.param("baseAngle", baseAngle, 0.0);
    pn.param("baseAngle", k_far, 1.0);

    //Publishers and Subscribers
    odom_sub = n_.subscribe("/odometry/filtered", 100, &L1Controller::odomCB, this);
    path_sub = n_.subscribe("/move_base/NavfnROS/plan", 10, &L1Controller::pathCB, this);
    //path_sub = n_.subscribe("/move_base/TebLocalPlannerROS/global_plan", 10, &L1Controller::pathCB, this);
    goal_sub = n_.subscribe("/move_base_simple/goal", 1, &L1Controller::goalCB, this);
    marker_pub = n_.advertise<visualization_msgs::Marker>("car_path", 1);
    three_point_pub = n_.advertise<visualization_msgs::Marker>("three_point", 10);
    pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel_plan", 10);

    //Timer
    timer1 = n_.createTimer(ros::Duration((1.0) / controller_freq), &L1Controller::controlLoopCB, this); // Duration(0.05) -> 20Hz
    timer2 = n_.createTimer(ros::Duration((0.5) / controller_freq), &L1Controller::goalReachingCB, this); // Duration(0.05) -> 20Hz
    timer3 = n_.createTimer(ros::Duration((0.5) / 40), &L1Controller::getodomCB, this);
    //Init variables
    Lfw = goalRadius = getL1Distance(Vcmd);
    foundForwardPt = false;
    goal_received = false;
    goal_reached = false;

    //Show info
    ROS_INFO("[param] baseSpeed: %f", baseSpeed);
    ROS_INFO("[param] baseAngle: %f", baseAngle);
    ROS_INFO("[param] AngleGain: %f", Angle_gain);
    ROS_INFO("[param] Vcmd: %f", Vcmd);
    ROS_INFO("[param] Lfw: %f", Lfw);

    //Visualization Marker Settings
    initMarker();
}



void L1Controller::initMarker()
{
    three_points.header.frame_id = "map";
    three_points.ns = "Markers";
    three_points.pose.orientation.w = 1.0;
    three_points.id = 3;
    three_points.type = visualization_msgs::Marker::SPHERE_LIST;
    three_points.scale.x = 0.2;
    three_points.scale.y = 0.2;
    three_points.color.g = 1.0f;
    three_points.color.a = 1.0;


    points.header.frame_id = line_strip.header.frame_id = goal_circle.header.frame_id = "odom";
    points.ns = line_strip.ns = goal_circle.ns = "Markers";
    points.action = line_strip.action = goal_circle.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = goal_circle.pose.orientation.w = 1.0;
    points.id = 0;
    line_strip.id = 1;
    goal_circle.id = 2;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    goal_circle.type = visualization_msgs::Marker::CYLINDER;
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    //LINE_STRIP markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;

    goal_circle.scale.x = goalRadius;
    goal_circle.scale.y = goalRadius;
    goal_circle.scale.z = 0.1;

    // Points are green
    three_points.color.b = 1.0;
    three_points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    //goal_circle is yellow
    goal_circle.color.r = 1.0;
    goal_circle.color.g = 1.0;
    goal_circle.color.b = 0.0;
    goal_circle.color.a = 0.5;
}

double L1Controller::transfrom_anglur(double anglur)
{
    int q = abs(anglur / (PI / 2));
    if (anglur < 0 && q % 2 == 1)
    {
        anglur = anglur + (q + 1) * PI / 2;
    }
    else if (anglur < 0 && q % 2 == 0)
    {
        anglur = anglur + q * PI / 2;
    }
    else if (anglur > 0 && q % 2 == 1)
    {
        anglur = anglur - (q + 1) * PI / 2;
    }
    else if ((anglur < 0 && q % 2 == 0))
    {
        anglur = anglur - q * PI / 2;
    }

    return anglur;
}
void L1Controller::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    odom = *odomMsg;
}
void L1Controller::sort(double* a, int length, int* b)
{
    int i, j;
    double t1, t;
    for (j = 0; j < length; j++)
        for (i = 0; i < length - 1 - j; i++)
            if (a[i] > a[i + 1])
            {
                t = a[i];
                a[i] = a[i + 1];
                a[i + 1] = t;


                t1 = b[i];
                b[i] = b[i + 1];
                b[i + 1] = t1;
            }
}

void L1Controller::pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
{
    map_path = *pathMsg;
}


void L1Controller::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
    try
    {
        geometry_msgs::PoseStamped odom_goal;
        tf_listener.transformPose("odom", ros::Time(0), *goalMsg, "map", odom_goal);
        odom_goal_pos = odom_goal.pose.position;
        goal_received = true;
        goal_reached = false;

        /*Draw Goal on RVIZ*/
        goal_circle.pose = odom_goal.pose;
        marker_pub.publish(goal_circle);
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
}

double L1Controller::curvature(Eigen::Matrix<double, 3, 1>& x, Eigen::Matrix<double, 3, 1>& y)
{
    double dx_1 = x(1, 0) - x(0, 0); double dx_2 = x(2, 0) - x(1, 0);
    double dy_1 = y(1, 0) - y(0, 0); double dy_2 = y(2, 0) - y(1, 0);
    double t_a = sqrt(dx_1 * dx_1 + dy_1 * dy_1);
    double t_b = sqrt(dx_2 * dx_2 + dy_2 * dy_2);
    Eigen::Matrix<double, 3, 3> M;
    M << 1, -t_a, t_a* t_a,
        1, 0, 0,
        1, t_b, t_b* t_b;
    Eigen::Matrix<double, 3, 1> a = M.inverse() * x;
    Eigen::Matrix<double, 3, 1> b = M.inverse() * y;
    double kappa = 2 * (a(2, 0) * b(1, 0) - a(1, 0) * b(2, 0)) / pow((pow(a(1, 0), 2) + pow(b(1, 0), 2)), 1.5);
    //std::cout<<"kappa:"<<kappa<<std::endl;
    return kappa;
}

double L1Controller::getYawFromPose(const geometry_msgs::Pose& carPose)
{
    float x = carPose.orientation.x;
    float y = carPose.orientation.y;
    float z = carPose.orientation.z;
    float w = carPose.orientation.w;

    double tmp, yaw;
    tf::Quaternion q(x, y, z, w);
    tf::Matrix3x3 quaternion(q);
    quaternion.getRPY(tmp, tmp, yaw);

    return yaw;
}

bool L1Controller::isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose)
{
    float car2wayPt_x = wayPt.x - carPose.position.x;
    float car2wayPt_y = wayPt.y - carPose.position.y;
    double car_theta = getYawFromPose(carPose);

    float car_car2wayPt_x = cos(car_theta) * car2wayPt_x + sin(car_theta) * car2wayPt_y;
    float car_car2wayPt_y = -sin(car_theta) * car2wayPt_x + cos(car_theta) * car2wayPt_y;

    if (car_car2wayPt_x > 0) /*is Forward WayPt*/
        return true;
    else
        return false;
}


bool L1Controller::isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos)
{
    double dx = wayPt.x - car_pos.x;
    double dy = wayPt.y - car_pos.y;
    double dist = sqrt(dx * dx + dy * dy);

    if (dist < Lfw)
        return false;
    else if (dist >= Lfw)
        return true;
}

geometry_msgs::Point L1Controller::get_odom_car2WayPtVec(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point carPose_pos = carPose.position;
    double carPose_yaw = getYawFromPose(carPose);
    geometry_msgs::Point forwardPt;
    geometry_msgs::Point odom_car2WayPtVec;
    foundForwardPt = false;

    if (!goal_reached) {
        for (int i = 0; i < map_path.poses.size(); i++)
        {
            geometry_msgs::PoseStamped map_path_pose = map_path.poses[i];
            geometry_msgs::PoseStamped odom_path_pose;

            try
            {
                tf_listener.transformPose("odom", ros::Time(0), map_path_pose, "map", odom_path_pose);
                geometry_msgs::Point odom_path_wayPt = odom_path_pose.pose.position;
                bool _isForwardWayPt = isForwardWayPt(odom_path_wayPt, carPose);

                if (_isForwardWayPt)
                {
                    bool _isWayPtAwayFromLfwDist = isWayPtAwayFromLfwDist(odom_path_wayPt, carPose_pos);
                    if (_isWayPtAwayFromLfwDist)
                    {
                        forwardPt = odom_path_wayPt;
                        foundForwardPt = true;
                        break;
                    }
                }
            }
            catch (tf::TransformException& ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }
        }

    }
    else if (goal_reached)
    {
        forwardPt = odom_goal_pos;
        foundForwardPt = false;
        //ROS_INFO("goal REACHED!");
    }

    /*Visualized Target Point on RVIZ*/
    /*Clear former target point Marker*/
    points.points.clear();
    line_strip.points.clear();

    if (foundForwardPt && !goal_reached)
    {
        points.points.push_back(carPose_pos);
        points.points.push_back(forwardPt);
        line_strip.points.push_back(carPose_pos);
        line_strip.points.push_back(forwardPt);
    }

    marker_pub.publish(points);
    marker_pub.publish(line_strip);

    odom_car2WayPtVec.x = cos(carPose_yaw) * (forwardPt.x - carPose_pos.x) + sin(carPose_yaw) * (forwardPt.y - carPose_pos.y);
    odom_car2WayPtVec.y = -sin(carPose_yaw) * (forwardPt.x - carPose_pos.x) + cos(carPose_yaw) * (forwardPt.y - carPose_pos.y);
    return odom_car2WayPtVec;
}


double L1Controller::getEta(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point odom_car2WayPtVec = get_odom_car2WayPtVec(carPose);

    double eta = atan2(odom_car2WayPtVec.y, odom_car2WayPtVec.x);
    return eta;
}


double L1Controller::getCar2GoalDist()
{
    geometry_msgs::Point car_pose = odom.pose.pose.position;
    double car2goal_x = odom_goal_pos.x - car_pose.x;
    double car2goal_y = odom_goal_pos.y - car_pose.y;

    double dist2goal = sqrt(car2goal_x * car2goal_x + car2goal_y * car2goal_y);

    return dist2goal;
}

double L1Controller::getL1Distance(const double& _Vcmd)
{
    double L1 = 0;
    if (_Vcmd < 1.34)
        L1 = 3 / 3.0;
    else if (_Vcmd > 1.34 && _Vcmd < 5.36)
        L1 = _Vcmd * 2.24 / 3.0;
    else
        L1 = 12 / 3.0;
    return L1;
}

double L1Controller::getSteeringAngle(double eta)
{
    double steeringAnge = -atan2((L * sin(eta)), (Lfw / 2 + lfw * cos(eta))) * (180.0 / PI);
    //ROS_INFO("Steering Angle = %.2f", steeringAnge);
    return steeringAnge;
}

double L1Controller::getGasInput(const float& current_v)
{
    double u = (Vcmd - current_v) * Gas_gain;
    //ROS_INFO("velocity = %.2f\tu = %.2f",current_v, u);
    return u;
}


void L1Controller::goalReachingCB(const ros::TimerEvent&)
{
    if (goal_received)
    {
        double car2goal_dist = getCar2GoalDist();
        if (car2goal_dist < goalRadius)
        {
            goal_reached = true;
            goal_received = false;
            ROS_INFO("Goal Reached !");

        }
    }
}
void L1Controller::getodomCB(const ros::TimerEvent&)
{
    tf::StampedTransform transform;
    try
    {
        listener.lookupTransform("map", "base_footprint", ros::Time(0), transform);
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    car_x = transform.getOrigin().x();
    car_y = transform.getOrigin().y();
    car_yaw = tf::getYaw(transform.getRotation());
}
void L1Controller::controlLoopCB(const ros::TimerEvent&)
{
    if(goal_reached==false)
    { 
    geometry_msgs::Pose pose;
    yaw = car_yaw;
    //std::cout<<yaw*180/PI<<std::endl;
    if (map_path.poses.size() > 0)
    {
        //计算曲率
        /*path_near_x << map_path.poses[0].pose.position.x,
            map_path.poses[22].pose.position.x,
            map_path.poses[45].pose.position.x;
        path_near_y << map_path.poses[0].pose.position.y,
            map_path.poses[22].pose.position.y,
            map_path.poses[45].pose.position.y;
        path_far_x << map_path.poses[45].pose.position.x,
            map_path.poses[62].pose.position.x,
            map_path.poses[100].pose.position.x;
        path_far_y << map_path.poses[45].pose.position.y,
            map_path.poses[62].pose.position.y,
            map_path.poses[100].pose.position.y;
        path_near_kappa = curvature(path_near_x, path_near_y);
        path_far_kappa = curvature(path_far_x, path_far_y);
        //std::cout<<"path_near_kappa:"<<path_near_kappa<<"  path_far_kappa:"<<path_far_kappa<<std::endl;*/
        //根据曲率设置参数

        //斯坦利算法

        //std::cout<<odom.pose.pose.position.x<<"  "<<odom.pose.pose.position.y<<std::endl;
        //std::cout<<odom_x<<"  "<<odom_y<<std::endl;
        //std::cout<<std::endl;


       /* if (abs(path_near_kappa) > 0.3 || abs(path_far_kappa) > 0.3)
        {
            //cmd_vel.linear.x = 0.5;
            
        }
        else
        {
            //cmd_vel.linear.x = 0.5;
            look_ahead = 0.3;
        }*/
       
        for (i = 0; i < num; i++)
        {
            follow_x = map_path.poses[i].pose.position.x;
            follow_y = map_path.poses[i].pose.position.y;
            min_distance[i] = sqrt(pow((follow_x - odom_x), 2) + pow((follow_y - odom_y), 2));
            b[i] = i;
        }
        sort(min_distance, num, b);
        index = b[0] + 2;
        follow_x = map_path.poses[index].pose.position.x;
        follow_y = map_path.poses[index].pose.position.y;
        dy = (map_path.poses[index + 1].pose.position.y - follow_y);
        dx = (map_path.poses[index + 1].pose.position.x - follow_x);
        ch = atan2(dy, dx);
        alpha = ch - yaw;
        if (alpha > PI)alpha = alpha - 2 * PI;
        else if (alpha < -PI)alpha = alpha + 2 * PI;
        else alpha = alpha;

        if (abs(alpha) > 0.2)
        {
            if (alpha > 0)
            {
                e = abs(sqrt(pow((follow_x - odom_x), 2) + pow((follow_y - odom_y), 2)));
                //std::cout<<"向左："<<e<<std::endl;
                //e=abs(sqrt(pow((map_path.poses[b[0]].pose.position.x-odom_x),2)+pow((map_path.poses[b[0]].pose.position.y-odom_y),2)));
                std::cout << "向左：" << e << std::endl;
            }
            else
            {
                e = -abs(sqrt(pow((follow_x - odom_x), 2) + pow((follow_y - odom_y), 2)));
                // std::cout<<"向右："<<e<<std::endl;
                 //e=-abs(sqrt(pow((map_path.poses[b[0]].pose.position.x-odom_x),2)+pow((map_path.poses[b[0]].pose.position.y-odom_y),2)));
                std::cout << "向右：" << e << std::endl;
            }
            k_stanley = 2.4;
			cmd_vel.linear.x = 0.5;
			look_ahead = 0.2;
        }
        else
        {
            std::cout << "走直线" << std::endl;
            if (atan((follow_y - car_y) / (follow_x - car_x)) > 0)
                // if(atan((follow_y-odom_y)/(follow_x-odom_x))>0)
                // if(alpha>0)
            {
                e = abs(sqrt(pow((map_path.poses[b[0]].pose.position.x - odom_x), 2) + pow((map_path.poses[b[0]].pose.position.y - odom_y), 2)));
                std::cout << "向左：" << e << std::endl;
            }
            else
            {
                e = -abs(sqrt(pow((map_path.poses[b[0]].pose.position.x - odom_x), 2) + pow((map_path.poses[b[0]].pose.position.y - odom_y), 2)));
                std::cout << "向右：" << e << std::endl;
            }
            k_stanley = 0.5;
			cmd_vel.linear.x = 0.8;
			look_ahead = 0.5;
        }

		odom_x = car_x + look_ahead * cos(yaw);
        odom_y = car_y + look_ahead * sin(yaw);

        delta_e = atan(k_stanley * e / cmd_vel.linear.x);//0.4


        theta = delta_e + alpha;


        //last_dalta=theta;
        //now_dalta


        std::cout << "delta_e:" << delta_e << "  alpha:" << alpha << std::endl;
        std::cout << "theta:" << theta << std::endl;

        pose.position.x = car_x;
        pose.position.y = car_y;



        //点的可视化
        three_points.action = visualization_msgs::Marker::ADD;
        three_points.points.push_back(map_path.poses[index].pose.position);
        three_points.points.push_back(pose.position);
        //three_points.points.push_back()；
           //three_points.points.push_back(map_path.poses[prospect*2].pose.position);
        three_point_pub.publish(three_points);
        three_points.points.clear();

        cmd_vel.angular.z = (theta);
        pub_.publish(cmd_vel);
	}	
    }
    else
    {
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
	pub_.publish(cmd_vel);
    }
 

}


/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char** argv)
{
    //Initiate ROS
    ros::init(argc, argv, "L1Controller_v2");
    L1Controller controller;
    ros::spin();
    return 0;
}