#include "command_pub.h"

#define NODE_NAME "command_pub"
using namespace std;
#define M_PI 3.14159265358979323846
#define simu_flag  1
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_sender");
    ros::NodeHandle nh("~");
    nh.param("uav_id", uav_id, 0);
    nh.param("neighbour_id1", neighbour_id1, 0);
    nh.param("neighbour_id2", neighbour_id2, 0);
    nh.param("controller_flag", controller_flag, 0);
    // 编队控制参数
    nh.param<float>("swarm_num", swarm_num, 1);
    nh.param<float>("k_p", k_p, 0.95);
    nh.param<float>("k_aij", k_aij, 0.1);
    nh.param<float>("k_gamma", k_gamma, 0.1);
    nh.param<float>("k_avoid", k_avoid, 0.1);
    nh.param<float>("pobs_x", pobs_x, 0.0);
    nh.param<float>("pobs_y", pobs_y, 0.0);
    //cout<<pobs_x<<pobs_y<<endl;
    uav_name = "/uav" + std::to_string(uav_id);
    neighbour_name1 = "/uav" + std::to_string(neighbour_id1);
    neighbour_name2 = "/uav" + std::to_string(neighbour_id2);
    //【订阅】控制指令
    ros::Subscriber Command_sub = nh.subscribe<geometry_msgs::PoseStamped>("/control_command", 10, Command_cb);
    Command_pub = nh.advertise<geometry_msgs::PoseStamped>("/command_pub", 10);
    //【订阅】无人机world坐标[optitrack]
    // ros::Subscriber drone_offboard = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/uav1/pose", 10, offset_cb);
    //【订阅】无人机world坐标[uwb]
    // ros::Subscriber drone_offboard = nh.subscribe<geometry_msgs::PoseStamped>(uav_name +"/uwb/pose", 10, drone_state_uwb_cb);
    // 【订阅】px4状态 - 来自飞控
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(uav_name +"/mavros/state", 10, state_cb);
    //【订阅】无人机world坐标
    ros::Subscriber drone_state_sub = nh.subscribe<nav_msgs::Odometry>(uav_name +"/drone_state", 10, drone_state_cb);
    // 【订阅】无人机formation
    ros::Subscriber formation_sub = nh.subscribe<state_msg::form>("/formation", 10, form_cb);
    //【发布】控制指令
    //setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/setpoint_raw/local", 10);
    setformation_pub = nh.advertise<state_msg::form>("/set_formation", 10);
    //【发布】控制指令
    //setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/setpoint_raw/local", 10);
    //【发布】控制指令
    // 发送-->px4
    setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>(uav_name +"/mavros/setpoint_raw/local", 10);
    attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>(uav_name +"/mavros/setpoint_raw/attitude", 1);
    //cmd_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("/cmd", 1);
    obs_pub_ = nh.advertise<visualization_msgs::Marker>("/obs", 1);
    //【订阅】邻居飞机的状态信息
    if(neighbour_name1 != "/uav1" || neighbour_name2 != "/uav1")
    {
        nei1_state_sub = nh.subscribe<nav_msgs::Odometry>(neighbour_name1 + "/drone_state", 10, boost::bind(&nei_state_cb,_1, 0));
        nei2_state_sub = nh.subscribe<nav_msgs::Odometry>(neighbour_name2 + "/drone_state", 10, boost::bind(&nei_state_cb,_1, 1));
    }
    // 建议控制频率 ： 10 - 50Hz, 控制频率取决于控制形式，若控制方式为速度或加速度应适当提高频率
    ros::Rate rate(50.0);
    ros::Duration(3.0).sleep();
    ros::spinOnce();
    Takeoff_position[0] = pos_drone[0];
    Takeoff_position[1] = pos_drone[1];
    Takeoff_position[2] = pos_drone[2];
    cout<<"无人机起飞位置： "<<Takeoff_position[0]<<" "<<Takeoff_position[1]<<" "<<Takeoff_position[2]<<endl;
    // 记录启控时间
    ros::Time begin_time = ros::Time::now();
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主  循  环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        ros::spinOnce();
        if(controller_flag == 0)
        {
            if(time_flag >= 0 && time_flag <= 100 )
                {
                    state_sp = Eigen::Vector3d(Takeoff_position[0],Takeoff_position[1],Command_Now.pose.position.z);
                    yaw_sp = quaternionToYaw(q_fcu);
                    yaw_sp = yaw_sp * (M_PI / 180.0);initial_velocity[0] = std::sqrt(2 * acceleration * distance_to_target[0]);
                    send_pos_setpoint(state_sp, yaw_sp);
                }
            else if(time_flag > 100 && time_flag <= 200)
                {
                    x_sp = Takeoff_position[0] + (Command_Now.pose.position.x -  Takeoff_position[0]) * (time_flag-100) /100;
                    y_sp = Takeoff_position[1] + (Command_Now.pose.position.y -  Takeoff_position[1]) * (time_flag-100) /100;
                    state_sp = Eigen::Vector3d(x_sp,y_sp,Command_Now.pose.position.z);
                    yaw_sp = quaternionToYaw(q_fcu);
                    yaw_sp = yaw_sp * (M_PI / 180.0);
                    send_pos_setpoint(state_sp, yaw_sp);
                }
            else if(time_flag > 200)
                {
                    ros::spinOnce();
                    state_sp = Eigen::Vector3d(Command_Now.pose.position.x,Command_Now.pose.position.y,Command_Now.pose.position.z);
                    error_flag = error_check(state_sp,state_des);
                    if(error_flag == 1)
                    {
                        state_sp = Eigen::Vector3d(Command_Now.pose.position.x,Command_Now.pose.position.y,Command_Now.pose.position.z);
                        yaw_sp = quaternionToYaw(q_fcu);
                        yaw_sp = yaw_sp * (M_PI / 180.0);
                    }
                    else{
                        time_flag = 0;
                        ros::spinOnce();
                        Takeoff_position[0] = offset_position[0];
                        Takeoff_position[1] = offset_position[1];
                    }
                    send_pos_setpoint(state_sp, yaw_sp);
                }
            else
                {
                    ROS_ERROR("time_flag error");
                }
            
        }

        else if(controller_flag == 1)
        {
            Eigen::Vector2d obs(pobs_x,pobs_y); 
            obsPublish(obs);
            if(time_flag >= 0 && time_flag <= 100)
            {
                state_des[0] = Takeoff_position[0];
                state_des[1] = Takeoff_position[1];   
                state_des[2] = Takeoff_position[2];
                send_pos_setpoint(state_des,0);
                //CmdPublish(state_des,Eigen::Vector3d(0,0,0),Eigen::Vector3d(0,0,0),Eigen::Vector3d(0,0,0));
            }
            else
            {
            
            yaw_sp = quaternionToYaw(q_fcu);
            yaw_sp = yaw_sp * (M_PI / 180.0);
            formation_separation = formation_utils::get_formation_separation(formation_msg.swarm_shape, formation_msg.swarm_size);
            yita = 1/ ((float)swarm_num * k_aij + k_p);
            state_des[0] = 0.0;
            state_des[1] = 0.0;
            state_des[2] = Command_Now.pose.position.z + formation_separation(uav_id-1,2);
            // 计算当前位置与目标点之间的距离
            distance_to_target[0] = Command_Now.pose.position.x + formation_separation(uav_id-1,0) - pos_drone[0];
            distance_to_target[1] = Command_Now.pose.position.y + formation_separation(uav_id-1,1) - pos_drone[1];
            // 计算以给定加速度达到目标点所需的初速度
            initial_velocity[0] = (distance_to_target[0] > 0 ? 1 : -1)*std::sqrt(2 * acceleration * abs(distance_to_target[0]));  
            initial_velocity[1] = (distance_to_target[1] > 0 ? 1 : -1)*std::sqrt(2 * acceleration * abs(distance_to_target[1]));
            //避障
            if(obs[0] !=0 || obs[1] !=0){
                avoid.pos_drone_avoid<<pos_drone[0],pos_drone[1];
                avoid.pos_target<<Command_Now.pose.position.x + formation_separation(uav_id-1,0),Command_Now.pose.position.y + formation_separation(uav_id-1,1);
                avoid.pos_obs1<<obs[0],obs[1];
                avoid.process();
                //cout<<"avoid:"<<avoid.v0[0]<<","<<avoid.v0[1]<<endl;
            }
            else{
                avoid.v0.setZero();
            }
            //cout<<"initial_velocity:"<<initial_velocity[0]<<","<<initial_velocity[1]<<endl;
            vel_des[0] = - yita * k_aij * ( vel_nei[0][0] - k_gamma *((pos_drone[0] - pos_nei[0][0]) - ( formation_separation(uav_id-1,0) -  formation_separation(neighbour_id1-1,0)))) 
                        - yita * k_aij * ( vel_nei[1][0] - k_gamma *((pos_drone[0] - pos_nei[1][0]) - ( formation_separation(uav_id-1,0) -  formation_separation(neighbour_id2-1,0))))
                        + yita * k_p * ( initial_velocity[0] - k_gamma * (pos_drone[0] - Command_Now.pose.position.x - formation_separation(uav_id-1,0))) + k_avoid * avoid.v0[0];
            vel_des[1] = - yita * k_aij * ( vel_nei[0][1] - k_gamma *((pos_drone[1] - pos_nei[0][1]) - ( formation_separation(uav_id-1,1) -  formation_separation(neighbour_id1-1,1)))) 
                        - yita * k_aij * ( vel_nei[1][1] - k_gamma *((pos_drone[1] - pos_nei[1][1]) - ( formation_separation(uav_id-1,1) -  formation_separation(neighbour_id2-1,1))))
                        + yita * k_p * ( initial_velocity[1] - k_gamma * (pos_drone[1] - Command_Now.pose.position.y - formation_separation(uav_id-1,1))) + k_avoid * avoid.v0[1];
            vel_des[2] = 0.0;
            acc_des << 0.0, 0.0, 0.0;
            yaw_des = yaw_sp + formation_separation(uav_id-1,3); 
            send_vel_xy_pos_z_setpoint(state_des, vel_des, yaw_des);
            //CmdPublish(pos_drone,vel_des,acc_des,Eigen::Vector3d(0,0,0));
            } 
            setformation_pub.publish(formation_msg);
            Command_pub.publish(Command_Now);
        }
        else if(controller_flag == 2)
        {
            if(time_flag >= 0 && time_flag <= 100)
            {
            state_des[0] = Takeoff_position[0];
            state_des[1] = Takeoff_position[1];
            state_des[2] = Command_Now.pose.position.z;  
            
            }
            else
            {      
            formation_separation = formation_utils::get_formation_separation(formation_msg.swarm_shape, formation_msg.swarm_size);
            state_des[0] = Command_Now.pose.position.x + formation_separation(uav_id-1,0);
            state_des[1] = Command_Now.pose.position.y + formation_separation(uav_id-1,1);
            state_des[2] = Command_Now.pose.position.z + formation_separation(uav_id-1,2);
            //cout<<"pos_target: "<<state_des[0]<<" "<<state_des[1]<<" "<<state_des[2]<<endl;
            }
            setformation_pub.publish(formation_msg);
            Command_pub.publish(Command_Now);
            send_pos_setpoint(state_des,0);
        } 
        if(mode == "OFFBOARD"||simu_flag == 1)
        {
            time_flag += 1;
        }
        rate.sleep();
    }
    return 0;
}
