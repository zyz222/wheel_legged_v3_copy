#ifndef _RobotState
#define _RobotState
#pragma once  
#include <eigen3/Eigen/Dense>
#include "convexMPC/common_types.h"

using Eigen::Matrix;
using Eigen::Quaternionf;

#define MPC_PERIOD_LEN 10   //模型预测步长
#define FAST_MPC 1

void RobotState_set(flt* p, flt* v, flt* q, flt* w, flt* r, flt yaw);   //位置、速度 、姿态、角速度、足端位置、航向角
void RobotState_print();
typedef struct{
    
    Matrix<fpt,3,1> p,v,w;    //位置、速度、角速度 三维  float类型的
    Matrix<fpt,3,4> r_feet;   //足端反力
    Matrix<fpt,3,3> R;       //旋转矩阵
    Matrix<fpt,3,3> R_yaw;     //yaw旋转矩阵
    Matrix<fpt,3,3> I_body;         //惯性矩阵
    Quaternionf q;      //四元数
    fpt yaw;             //航向角
    fpt m = 26.135;    //机身质量，不算腿部
// fpt m = 41.598;   //整机质量
    
}RobotState;

extern RobotState robot_rs;
#endif
