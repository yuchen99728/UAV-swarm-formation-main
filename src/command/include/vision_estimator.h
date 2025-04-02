#pragma once
#include <ros/ros.h>
#include <iostream>
#include <bitset>
#include <Eigen/Eigen>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <std_msgs/Float64.h>
using namespace std;
int uav_id;
int input_source;
double uav_init_x;
double uav_init_y;
string uav_name; 
Eigen::Vector3d pos_drone[2];             
Eigen::Vector3d vel_drone[2]; 
Eigen::Vector3d pos_drone_t265;
Eigen::Quaterniond q_t265;
Eigen::Vector3d  mocap_position;
Eigen::Quaterniond q_mocap;
Eigen::Vector3d uwb_position;
Eigen::Quaterniond q_uwb;
ros::Publisher vision_pub;
ros::Publisher drone_state_pub;
ros::Subscriber t265_sub;
ros::Subscriber velocity_sub;
ros::Subscriber pos_sub;
nav_msgs::Odometry _DroneState;
ros::Subscriber drone_offset;
ros::Timer timer_vision_pub;
ros::Subscriber drone_offboard; 
void t265_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (msg->header.frame_id == "t265_odom_frame")
    {
        pos_drone_t265 = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        q_t265 = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    }
    else
    {
        ROS_WARN("wrong t265 frame id.");
    }
}
void mocap_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    mocap_position = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    q_mocap = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
}

void uwb_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    uwb_position = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    //四元数这边先不写,xiami
    // pose_stamped.pose.orientation.w = 1;
    // pose_stamped.pose.orientation.x = 0;
    // pose_stamped.pose.orientation.y = 0;
    // pose_stamped.pose.orientation.z = 0; 
}

void timercb_vision_mocap(const ros::TimerEvent &e)
{
    geometry_msgs::PoseStamped vision;
    vision.pose.position.x = mocap_position[0] + uav_init_x;
    vision.pose.position.y = mocap_position[1] + uav_init_y;
    vision.pose.position.z = mocap_position[2];
    vision.pose.orientation.x = q_mocap.x();
    vision.pose.orientation.y = q_mocap.y();
    vision.pose.orientation.z = q_mocap.z();
    vision.pose.orientation.w = q_mocap.w();
    vision.header.stamp = ros::Time::now();
    vision_pub.publish(vision);
}

void timercb_vision(const ros::TimerEvent &e)
{
    geometry_msgs::PoseStamped vision;
    vision.pose.position.x = pos_drone_t265[0] + uav_init_x;
    vision.pose.position.y = pos_drone_t265[1] + uav_init_y;
    vision.pose.position.z = pos_drone_t265[2];
    vision.pose.orientation.x = q_t265.x();
    vision.pose.orientation.y = q_t265.y();
    vision.pose.orientation.z = q_t265.z();
    vision.pose.orientation.w = q_t265.w();
    vision.header.stamp = ros::Time::now();
    vision_pub.publish(vision);
}

void timercb_vision_uwb(const ros::TimerEvent &e)
{
    geometry_msgs::PoseStamped vision;
    vision.pose.position.x = uwb_position[0] + uav_init_x;
    vision.pose.position.y = uwb_position[1] + uav_init_y;
    vision.pose.position.z = uwb_position[2];
    //四元数这边注意一下,uwb本身是没有的
    // vision.pose.orientation.x = q_uwb.x();
    // vision.pose.orientation.y = q_uwb.y();
    // vision.pose.orientation.z = q_uwb.z();
    // vision.pose.orientation.w = q_uwb.w();
    vision.header.stamp = ros::Time::now();
    vision_pub.publish(vision);
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    _DroneState.pose.pose.position.x = msg->pose.position.x;
    _DroneState.pose.pose.position.y = msg->pose.position.y;
    _DroneState.pose.pose.position.z = msg->pose.position.z;
}
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    _DroneState.twist.twist.linear.x = msg->twist.linear.x;
    _DroneState.twist.twist.linear.y = msg->twist.linear.y;
    _DroneState.twist.twist.linear.z = msg->twist.linear.z;
}
void drone_state_cb(const ros::TimerEvent &e)
{
    _DroneState.header.stamp = ros::Time::now();
    _DroneState.header.frame_id = "world";
    drone_state_pub.publish(_DroneState);
}

