#pragma once
#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include "formation.h"
class DistributedControl{
private:

public:
    DistributedControl(ros::NodeHandle &nh){
        nh.param("/Dim", Dim, 1);
        nh.param("/Kp", Kp, 0.7);
        nh.param("/Kv", Kv, 0.1);
        nh.param("/control_flag", control_flag, 1);
        nh.param("/neighbour_id1", neighbour_id1, 0);
        nh.param("/neighbour_id2", neighbour_id2, 0);
        nh.param("/uav_id", uav_id, 1);
    }
    ~DistributedControl();
    void run(){
        W.setIdentity();
        for(int i=0;i<Dim;i++){
            for(int j=0;j<Dim;j++){
                if(i!=j && neighbour_check(i,j,neighbour_id1,neighbour_id2))
                {
                    W(i,j)= 1;
                }
            }
        }
        switch(control_flag){
            case 1:
                //case 1: distributed control
                break;
            case 2:
                //case 2: centralized control
                break;
            case 3:
                //case 3: consensus control
                break;
            case 4:
               for(int i=0;i<Dim;i++){
                    for(int j=0;j<Dim;j++){
                      acc.col(i) += - Kp * W(i,j)* (state_des.col(i) -  state_des.col(j)) - Kv * W(i,j)* (vel_des.col(i) -  vel_des.col(j));
                       
                    }
                    acc_des.col(i) += acc.col(i);
                }
                for(int i=0;i<Dim;i++){
                    for(int j=0;j<Dim;j++){
                        if(j!=i )
                        {
                            acc1.col(i) += W(i,j)*acc.col(j)
                            // acc1.col(i) +=  W(i,j)* (.col(i) -  state_des.col(j));
                        }
                    }
                    acc_des.col(i) += acc1.col(i)/W.sum();
                }
                U = acc_des.col(id-1);
                break;
            default:
                break;
        }


    }
public:
    int Dim,control_flag,neighbour_id1,neighbour_id2;
    float Kp,Kv;
    bool neighbour_check(int &neighbour_id1,int &neighbour_id2)
    {
        if(j+1 == neighbour_id1 || j+1 == neighbour_id2)
        {
            return true;
        }
        else if(neighbour_id1 == 0 && neighbour_id2 == 0)
        {
            ROS_INFO("single agent");
            return false;
        }
        else 
        {
            ROS_INFO("neighbour_id is not right set");
            return false;
        }
    }
    Eigen::MatrixXd W(Dim,Dim);
    Eigen::MatrixXd state_des(3,Dim);
    Eigen::MatrixXd vel_des(3,Dim);      
    Eigen::MatrixXd acc_des(3,Dim);
    Eigen::MatrixXd acc(3,Dim);
    Eigen::MatrixXd acc1(3,Dim);
    Eigen::Vector3d U;


    
};