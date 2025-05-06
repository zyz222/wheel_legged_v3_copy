// #pragma once
#include "convexMPC/RobotState.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>
// 在这里设置机器人的状态
using std::cout;
using std::endl;
RobotState robot_rs;
void RobotState_set(flt* p_, flt* v_, flt* q_, flt* w_, flt* r_,flt yaw_)
{
    for(u8 i = 0; i < 3; i++)
    {
        robot_rs.p(i) = p_[i];    //位置
        robot_rs.v(i) = v_[i];    //速度
        robot_rs.w(i) = w_[i];    //角速度
    }
    robot_rs.q.w() = q_[0];      
    robot_rs.q.x() = q_[1];
    robot_rs.q.y() = q_[2];
    robot_rs.q.z() = q_[3];
    robot_rs.yaw = yaw_;


    for(u8 rs = 0; rs < 3; rs++)
        for(u8 c = 0; c < 4; c++)
            robot_rs.r_feet(rs,c) = r_[rs*4 + c];    //足端反力

    robot_rs.R_yaw = robot_rs.q.toRotationMatrix();    //四元数转旋转矩阵
    fpt yc = cos(yaw_);
    fpt ys = sin(yaw_);

    robot_rs.R_yaw <<  yc,  -ys,   0,
             ys,  yc,   0,
               0,   0,   1;

    Matrix<fpt,3,1> Id;

    Id << 0.13317f, 0.96411f, 0.91536f;    //机身的转动惯量，不算腿部质量
    // Id << 1.912710f, 2.994957f, 3.056951f; //加上腿部质量的转动惯量


    robot_rs.I_body.diagonal() = Id;
}

void RobotState_print()
{
   cout<<"Robot State:"<<endl<<"Position\n"<<robot_rs.p.transpose()
       <<"\nVelocity\n"<<robot_rs.v.transpose()<<"\nAngular Veloctiy\n"
       <<robot_rs.w.transpose()<<"\nRotation\n"<<robot_rs.R<<"\nYaw Rotation\n"
       <<robot_rs.R_yaw<<"\nFoot Locations\n"<<robot_rs.r_feet<<"\nInertia\n"<<robot_rs.I_body<<endl;
}



