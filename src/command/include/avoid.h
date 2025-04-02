#pragma once
#include<iostream>
#include<cmath>
#include<ctime>
#include<vector>
#include<string>
#include <Eigen/Dense>

using namespace std;
class ArtificialPotential//人工势场法
{
public:
	double disCal( Eigen::Vector2d& pos_drone,  Eigen::Vector2d& pos_target)//两点距离计算
	{
		return (pos_drone - pos_target).norm();
	}

	Eigen::Vector2d Fatt()//计算引力
	{
		Rho_att = disCal(pos_drone_avoid, pos_target);//与目标点距离
		double F_att = eta * Rho_att;//引力大小
		Eigen::Vector2d pFatt(F_att / Rho_att * (pos_target[0] - pos_drone_avoid[0]), F_att / Rho_att * (pos_target[1]- pos_drone_avoid[1]));
		return pFatt;
	}
	Eigen::Vector2d Fobs()//计算斥力
	{
		// for (int it = 0; it < 1; it++)//分别计算每个障碍物产生的合斥力
		// {
			double Rho_obs = disCal(pos_obs1, pos_drone_avoid);//与这个障碍物中心点的距离
			double Freo1 = 0.0;
			double Freo2 = 0.0;//这个障碍物产生的两个引力的大小

			if (Rho_obs >= Rho0)
			{
				Freo1 = 0.0;
				Freo2 = 0.0;
			}
			else
			{
				Freo1 = k * (1 / Rho_obs - 1 / Rho0) * pow(Rho_att, n) / pow(Rho_obs, 2);
				Freo2 = n * k / 2 * pow(1 / Rho_obs - 1 / Rho0, 2) * pow(Rho_att, n - 1);
			}

			// Eigen::Vector2d pFreo1((pos_drone_avoid[0] - pos_obs1[0]) / Rho_obs * (Freo1 + Rho_obs) + pos_obs1[0], (pos_drone_avoid[1] - pos_obs1[1]) / Rho_obs * (Freo1 + Rho_obs) + pos_obs1[1]);
			// Eigen::Vector2d pFreo2(Freo2 / Rho_att * ( pos_target[0] - pos_drone_avoid[0]) + pos_drone_avoid[0], Freo2 / Rho_att * ( pos_target[1]- pos_drone_avoid[1]) + pos_drone_avoid[1]);
			// newVector vFreo1(road.car0->pc, pFreo1);
			Eigen::Vector2d pFreo1((pos_drone_avoid[0] - pos_obs1[0]) / Rho_obs * Freo1  , (pos_drone_avoid[1] - pos_obs1[1]) / Rho_obs * Freo1 );
			Eigen::Vector2d pFreo2(Freo2 / Rho_att * ( pos_target[0] - pos_drone_avoid[0]) , Freo2 / Rho_att * ( pos_target[1]- pos_drone_avoid[1]));
			Eigen::Vector2d vFobs = pFreo1+ pFreo2;//计算这个障碍物的合斥力
		// }
		// Eigen::Vector2d vFobsTotal(0,0);
		// for (int i = 0; i < 1; i++)
		// {
		// 	vFobsTotal = vFobsTotal.addVector(vFobsTotal, vFreo[i]);
		// }
		return vFobs;
	}
	void process()//整个过程
	{
		Eigen::Vector2d vFatt = Fatt();//引力
		Eigen::Vector2d vFobsTotal = Fobs();//总障碍物斥力
		Eigen::Vector2d vFtotal = vFatt + vFobsTotal;//引力与斥力的合力
		v0 = vFtotal * delta_t;//每个时间间隔内的速度
		// v0 += vFtotal.length;//速度}
	}
public:
    Eigen::Vector2d pos_obs1;
    Eigen::Vector2d pos_target;
    Eigen::Vector2d pos_drone_avoid;
	double Rho_att;//与目标点的距离
	Eigen::Vector2d vFreo;//存储每个障碍物产生的合斥力
	Eigen::Vector2d v0;//起始速度为0
private:
	double delta_t = 0.02;//时间步长,50hz
	double eta = 0.3;//引力增益系数
	double k = 0.3;//障碍物斥力增益系数
	int n = 3;
	double Rho0 = 1.0;//斥力起作用的范围,即障碍物膨胀半径
	
};

