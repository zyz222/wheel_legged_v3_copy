/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#ifndef WHEELTEST_H
#define WHEELTEST_H
#pragma once

#include "FSMState.h"
#include <ros/ros.h>
// #include "thirdParty/quadProgpp/QuadProg++.hh"
#include <osqp/osqp.h>
#include <geometry_msgs/Twist.h>
#include "control/BalanceCtrl.h"
#include <Eigen/Sparse>
using namespace Eigen;

template <typename T>
class State_WheelTest : public FSMState<T>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    State_WheelTest(CtrlComponents<T> *ctrlComp);
    ~State_WheelTest(){}
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();
    void setHighCmd(T vx, T vy, T wz);
    void mpc_init();
    void WheelControl();
    void update_mpc_state();
    void update_trajectory();
    void _calcCmd(Vec34<T> vecOP);
    Vec34<T> _calcOP(T row, T pitch, T yaw, T height);
private:
    void getUserCmdwheel();
    void initRecv();
    void twistCallback(const geometry_msgs::Twist& msg);
    ros::NodeHandle _nm;
    ros::Subscriber _cmdSub;          //订阅输入速度指令话题
    T _vx, _vy;                  //订阅话题速度指令
    T _wz;
    void calcTau();
    Vec3<T> _vCmdGlobal, _vCmdBody;        //机身速度指令
    T _yawCmd, _dYawCmd;
    Estimator<T> *_est;
    QuadrupedRobot<T> *_robModel;
    BalanceCtrl<T> *_balCtrl;
    Vec4<T> *_contact;

    RotMat<T> _Rd, _RdInit;        //旋转矩阵
    Vec3<T> _pcd, _pcdInit;
    T _kpw;
    Mat3<T> _Kpp, _Kdp, _Kdw;
    Vec3<T> _ddPcd, _dWbd;

    Vec12<T> _q, _tau;
    Vec3<T> _posBody, _velBody;
    RotMat<T> _B2G_RotMat, _G2B_RotMat;
    Vec34<T> _posFeet2BGlobal;
    Vec34<T> _forceFeetGlobal, _forceFeetBody;

    T _xMax, _xMin;
    T _yMax, _yMin;
    T _zMax, _zMin;
    T _yawMax, _yawMin;
    //mpc控制参数
    int n ;  //轮式运动控制MPC状态维度
    int p ;  //轮子运动控制MPC控制输入维度
    int Np ; //MPC预测步数
    T dt ; ///MPC采样时间间隔
    T theta_k ; // 初始偏航角 状态变量
    T x,y,z ;     //状态变量初始化
    T v,omega ;  //状态变量初始化
    T f1,f2,f3,f4 ;//控制变量初始化
    T m ;      //机身质量
    T J;   //绕Z轴转动惯量
    T L;   //轮距宽0.4m
    T g = 9.81 ;
    T mu = 0.5;
    T wheel_r = 0.08;
    T friction_torque;
    int wheel_nums;
    Mat5<T> A_state; //状态空间矩阵   A
    Vec54<T> B_ctrl; //控制输入空间矩阵   B
   
    std::vector<DMat<T>> A_powers;       //预计算A的N次幂矩阵
    MatX<T> Phi;         //填充fai矩阵
    DMat<T> Gamma ;
    DMat<T> A_power_B;
    //权重矩阵
    Mat5<T> S;       //终端权重矩阵
    Mat5<T> Q;      //运行过程权重矩阵
    HomoMat<T> R;     //控制输入权重矩阵   4*4
    DMat<T> Omega;
    DMat<T> Psi;
    DVec<T> x0;
    DVec<T> Xref;
    DVec<T> Uref;
    DVec<T> diffX;
    DMat<double> H,A_eq;
    DVec<double> f;
    DVec<double> lbU;
    DVec<double> ubU;
    SparseMatrix<double> H_sparse;
    SparseMatrix<double> A_eq_sparse;
    // DVec<T> l,u;
    DVec<double> U_opt;
    OSQPCscMatrix P_csc;
    OSQPCscMatrix A_csc;
    OSQPFloat* F = nullptr;
    OSQPFloat* l = nullptr;
    OSQPFloat* u = nullptr;
    OSQPSolver* solver  = nullptr;
    OSQPSettings* settings = nullptr;  
    /*位置环控制用 */
    Vec3<T> _initVecOX;
    Vec34<T> _initVecXP;
    T _rowMax, _rowMin;
    T _pitchMax, _pitchMin;
    T _yawMax_p, _yawMin_p;
    T _heightMax, _heightMin;

};



#endif /* WHEELTEST_H */
