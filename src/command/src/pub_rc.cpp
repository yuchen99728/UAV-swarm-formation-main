#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <nav_msgs/Path.h>
#include <state_msg/form.h>
#define NODE_NAME "terminal_control"
using namespace std;
//即将发布的command
geometry_msgs::PoseStamped Command_to_pub;
//发布
ros::Publisher move_pub;
ros::Publisher formation_pub;
Eigen::Quaterniond q_des;
int swarm_shape; //swarm_shape=0:one_column,swarm_shape=1:triangle , swarm_shape=2:square
float swarm_size;
state_msg::form formation_msg;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>　函数声明　<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void mainloop1();
Eigen::Quaterniond yaw_to_quaternion(double yaw)
{
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    return q;
}
void rc_callback(const mavros_msgs::RCIn::ConstPtr &msg)
{
    if(msg->channels[0] / 1000.0 == 0){
        swarm_shape = 0;
    }
    else if(msg->channels[0] / 1000.0 == 1){
        swarm_shape = 1;
    }
    else if(msg->channels[0] / 1000.0 == 2){
        swarm_shape = 2;
    }
    else{
        swarm_shape = 1;
    }
   
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>　主函数　<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "terminal_control");
    ros::NodeHandle nh;
    //　【发布】　控制指令
    move_pub = nh.advertise<geometry_msgs::PoseStamped>("control_command", 10);
    formation_pub = nh.advertise<state_msg::form>("formation", 10);
    ros::Subscriber rc = nh.subscribe("/uav1/mavros/rc/in", 10, rc_callback);
    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位https://wiki.amovlab.com/public/prometheuswiki/
    cout<<setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    mainloop1();
    return 0;
}
void mainloop1()
{
    float state_desired[4];
    while(ros::ok())
    {
        ros::spinOnce();
        swarm_size = 1;
        state_desired[0] = 0.0;
        state_desired[1] = 0.0;
        state_desired[2] = 0.8;
        state_desired[3] = 0.0;
        formation_msg.swarm_shape = swarm_shape;
        formation_msg.swarm_size = swarm_size;
        q_des = yaw_to_quaternion(state_desired[3]);
        Command_to_pub.header.stamp = ros::Time::now();
        Command_to_pub.pose.position.x = state_desired[0];
        Command_to_pub.pose.position.y = state_desired[1];
        Command_to_pub.pose.position.z = state_desired[2];
        Command_to_pub.pose.orientation.w = q_des.w();
        Command_to_pub.pose.orientation.x = q_des.x();
        Command_to_pub.pose.orientation.y = q_des.y();
        Command_to_pub.pose.orientation.z = q_des.z();        
        move_pub.publish(Command_to_pub);  
        formation_pub.publish(formation_msg);      
        cout << "....................................................." <<endl;
        sleep(1.0);
    }
}

