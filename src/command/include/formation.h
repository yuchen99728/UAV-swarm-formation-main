#ifndef FORMATION_UTILS_H
#define FORMATION_UTILS_H

#include <Eigen/Eigen>
#include <math.h>
#include <math_utils.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

using namespace Eigen;

float one_column_shape_for_3uav[12] = {
    1.5,0.0,0.0,0.0,    -1.5,0.0,0.0,0.0,   0.0,0.0,0.0,0.0};

float triangle_shape_for_3uav[12] = {
    1.0,0.0,0.0,0.0,   -0.6,-0.8,0.0,0.0,   -0.6,0.8,0.0,0.0};

float square_shape_for_3uav[12] = {
    1.0,1.0,0.0,0.0,   -1.0,1.0,0.0,0.0,   1.0,-1.0,0.0,0.0};
namespace formation_utils 
{
// 输入参数：　阵型，阵型基本尺寸，集群数量
// 所有的阵型和数量必须提前预设!!
Eigen::MatrixXf get_formation_separation(int swarm_shape, float swarm_size)
{
    //矩阵大小为　swarm_num＊4 , 对应x,y,z,yaw四个自由度的分离量
    Eigen::MatrixXf seperation(3,4); 
    // cxy 默认swarm_size为１米
    // 纵向一字型，虚拟领机位置为中心位置，其余飞机根据数量向左右增加
    if(swarm_shape == 0)
    {
        seperation = Map<Matrix<float,3,4,RowMajor>>(one_column_shape_for_3uav); 
    }
    // 三角型，虚拟领机位置为中心位置
    if(swarm_shape == 1)
    {
        seperation = Map<Matrix<float,3,4,RowMajor>>(triangle_shape_for_3uav); 
    }

    // 方型，虚拟领机位置为中心位置
    if(swarm_shape == 2)
    {
        seperation = Map<Matrix<float,3,4,RowMajor>>(square_shape_for_3uav); 
    }

    for(int i = 0 ; i < 3 ; i++)
    {
        for(int j = 0 ; j < 4; j++)
        {
            seperation(i,j) *= swarm_size;
        }
    }

    return seperation;
}
}
#endif