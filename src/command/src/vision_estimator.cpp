#include "vision_estimator.h"

using namespace std;
#define NODE_NAME "swarm_estimator" 

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh("~");
    // 读取参数
    nh.param("uav_id", uav_id, 0);
    nh.param("uav_init_x", uav_init_x, 0.0);
    nh.param("uav_init_y", uav_init_y, 0.0);
    nh.param("input_source", input_source, 1);
    uav_name = "/uav" + std::to_string(uav_id);
    //uav_name = "";
    // 订阅】t265估计位置
    t265_sub = nh.subscribe<nav_msgs::Odometry>(uav_name +"/t265/odom/sample", 100, t265_cb);
    //【订阅】无人机world坐标[optitrack]
    drone_offboard = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node"+uav_name+"/pose", 10, mocap_cb);
    
    //【订阅】无人机UWB信息
    drone_offset = nh.subscribe<geometry_msgs::PoseStamped>(uav_name +"/uwb_pose", 10, uwb_cb);

    vision_pub = nh.advertise<geometry_msgs::PoseStamped>(uav_name + "/mavros/vision_pose/pose", 100);
    pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(uav_name +"/mavros/local_position/pose", 10, pos_cb);
    velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>(uav_name +"/mavros/local_position/velocity_local", 10, vel_cb);
    //[发布】无人机state 坐标系 ENU系
    drone_state_pub = nh.advertise<nav_msgs::Odometry>(uav_name +"/drone_state", 10);
   
    //定时器,定时发送vision信息至飞控,保证50Hz以上
    if(input_source == 1)
    {
        timer_vision_pub = nh.createTimer(ros::Duration(0.02), timercb_vision_mocap);
    }
    if(input_source == 2)
    {
        timer_vision_pub = nh.createTimer(ros::Duration(0.02), timercb_vision);
    }
    if(input_source == 3)
    {
        
    }
    if(input_source == 4)
    {
        timer_vision_pub = nh.createTimer(ros::Duration(0.02), timercb_vision_uwb); 
    }
    ros::Timer timer_state_pub = nh.createTimer(ros::Duration(0.02), drone_state_cb);
    // 频率（这里频率是100？）
    ros::Rate rate(100.0);
    // ros::Rate rate(50.0);
    ros::spinOnce();
    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while (ros::ok())
    {
        //回调一次 更新传感器状态
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}