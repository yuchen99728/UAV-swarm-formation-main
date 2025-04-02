#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
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
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>　主函数　<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "terminal_control");
    ros::NodeHandle nh;
    //　【发布】　控制指令
    move_pub = nh.advertise<geometry_msgs::PoseStamped>("control_command", 10);
    formation_pub = nh.advertise<state_msg::form>("formation", 10);
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
        // Waiting for input
        cout << ">>>>>>>>>>>>>>>> Welcome to use Prometheus Terminal Control <<<<<<<<<<<<<<<<"<< endl;
        cout << "Please input the reference state [x y z yaw]: "<< endl;
        cout << "setpoint_t[0] --- x [m] : "<< endl;
        cin >> state_desired[0];
        cout << "setpoint_t[1] --- y [m] : "<< endl;
        cin >> state_desired[1];
        cout << "setpoint_t[2] --- z [m] : "<< endl;
        cin >> state_desired[2];
        cout << "setpoint_t[3] --- yaw [du] : "<< endl;
        cin >> state_desired[3];
        cout<<"input the formation : 0.one_column_shape 1.triangle_shape 2.square_shape (for 3 UAV)"<<endl;
        cin>>swarm_shape;
        cout<<"input the formation size: (default:1)"<<endl;
        cin>>swarm_size;
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

