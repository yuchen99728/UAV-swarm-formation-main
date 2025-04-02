#pragma once
#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <state_msg/form.h>
#include"avoid.h"
#include "formation.h" 
using namespace std;
#define M_PI 3.14159265358979323846
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>变量声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
Eigen::Vector3d Takeoff_position;   
Eigen::Vector3d offset_position(0,0,0);// 起飞位置
Eigen::Vector3d state_sp(0,0,0);
Eigen::Vector3d state_des(0,0,0);
Eigen::Vector3d vel_des(0,0,0);         
Eigen::Vector3d acc_des(0,0,0);  
Eigen::Quaterniond q_fcu;
// 无人机状态量
Eigen::Vector3d pos_drone;             
Eigen::Vector3d vel_drone; 
// 邻居状态量
Eigen::Vector3d pos_nei[2];                     // 邻居位置
Eigen::Vector3d vel_nei[2];                     // 邻居速度
// 编队控制相关
Eigen::MatrixXf formation_separation;           // 阵型偏移量
float k_p;                                      // 速度控制参数
float k_aij;                                    // 速度控制参数
float k_avoid;
float k_gamma;                                  // 速度控制参数
float yita;                                     // 速度控制参数
float swarm_num;                                // 群体数量
float pobs_x;
float pobs_y;
double yaw_sp = 0;
double yaw_des = 0;
double x_sp;
double y_sp;
Eigen::Vector2d distance_to_target;
Eigen::Vector2d initial_velocity;
ArtificialPotential avoid;
double acceleration = 0.25; // 加速度
ros::Publisher setpoint_raw_local_pub;
ros::Publisher attitude_pub;
ros::Publisher setformation_pub;
ros::Publisher Command_pub;
ros::Publisher cmd_pub_ ;
ros::Publisher obs_pub_;
geometry_msgs::PoseStamped  Command_Now;
state_msg::form formation_msg;
string uav_name;
string neighbour_name1;
string neighbour_name2;
int uav_id;
int neighbour_id1;
int neighbour_id2;
int time_flag = 0;
int controller_flag;
int swarm_shape;
bool error_flag =  false;
string mode;
ros::Subscriber nei1_state_sub;
ros::Subscriber nei2_state_sub;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void Command_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    Command_Now = *msg;
    q_fcu = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    state_des = Eigen::Vector3d(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
}

void send_pos_setpoint(const Eigen::Vector3d& pos_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
    pos_setpoint.coordinate_frame = 1;
    pos_setpoint.position.x = pos_sp[0];
    pos_setpoint.position.y = pos_sp[1];
    pos_setpoint.position.z = pos_sp[2];
    pos_setpoint.yaw = yaw_sp;
    setpoint_raw_local_pub.publish(pos_setpoint);
}
void send_vel_xy_pos_z_setpoint(const Eigen::Vector3d& pos_sp, const Eigen::Vector3d& vel_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;
    pos_setpoint.type_mask = 0b100111000011;   // 100 111 000 011  vx vy vz z + yaw

    pos_setpoint.coordinate_frame = 1;
    pos_setpoint.position.x = pos_sp[0];
    pos_setpoint.position.y = pos_sp[1];
    pos_setpoint.position.z = pos_sp[2];
    pos_setpoint.velocity.x = vel_sp[0];
    pos_setpoint.velocity.y = vel_sp[1];
    pos_setpoint.velocity.z = 0.0;
    pos_setpoint.yaw = yaw_sp;

    setpoint_raw_local_pub.publish(pos_setpoint);
}
// 函数：将四元数转换为yaw角度
double quaternionToYaw(const Eigen::Quaterniond& q) {
    // 计算yaw角度，仅考虑绕z轴的旋转
    double yaw = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    // 将弧度转换为角度
    yaw = yaw * 180.0 / M_PI;
    // 确保yaw在-180到180度之间
    if (yaw > 180.0) {
        yaw -= 360.0;
    } else if (yaw <= -180.0) {
        yaw += 360.0;
    }
    return yaw;
}
bool error_check(const Eigen::Vector3d& pos, const Eigen::Vector3d& des_pos)
{
    double x_error = fabs(pos[0] - des_pos[0]);
    double y_error = fabs(pos[1] - des_pos[1]);
    if(x_error < 0.1 && y_error < 0.1)
        return true;
    else
        return false;
}
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    mode = msg->mode;
}
void form_cb(const state_msg::form::ConstPtr& msg)
{   
    formation_msg.swarm_shape = msg->swarm_shape;
    formation_msg.swarm_size = msg->swarm_size;
}
void nei_state_cb(const nav_msgs::Odometry::ConstPtr& msg, int nei_id)
{
    pos_nei[nei_id]  = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    vel_nei[nei_id]  = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
}
void drone_state_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    pos_drone  = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    vel_drone  = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
}
//【自己加的】
// void drone_state_uwb_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
// {
//     pos_drone  = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
//     vel_drone  = Eigen::Vector3d(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
// }

void obsPublish(Eigen::Vector2d& nodes) {
        visualization_msgs::Marker node_vis; 
        node_vis.header.frame_id = "world";
        node_vis.header.stamp = ros::Time::now();
        node_vis.ns = "obs";
        node_vis.color.a = 0.7;
        node_vis.color.r = 0.5;
        node_vis.color.g = 0.5;
        node_vis.color.b = 0.5;
        node_vis.type = visualization_msgs::Marker::CUBE_LIST;
        node_vis.action = visualization_msgs::Marker::ADD;//rviz添加可视立方体图形
        node_vis.id = 0;
        node_vis.pose.orientation.x = 0.0;
        node_vis.pose.orientation.y = 0.0;
        node_vis.pose.orientation.z = 0.0;
        node_vis.pose.orientation.w = 1.0;
        
        node_vis.scale.x = 0.5;
        node_vis.scale.y = 0.5;
        node_vis.scale.z = 2.0;

        geometry_msgs::Point pt;
        pt.x = nodes(0);
        pt.y = nodes(1);
        pt.z = 0.0;
        node_vis.points.push_back(pt);
        obs_pub_.publish(node_vis);
    }