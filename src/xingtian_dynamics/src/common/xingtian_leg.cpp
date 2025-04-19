/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "common/xingtian_leg.h"
#include <iostream>
#include <cmath>
using namespace std;
/************************/
/*******QuadrupedLeg*****/
/************************/
//LF LR RR RF  HIP KNEE WHEEL
template <typename T>
QuadrupedLeg<T>::QuadrupedLeg(int legID, T hipLinkLength, 
                           T kneeLinkLength, Vec3<T> pHip2B)
            :_hipLinkLength(hipLinkLength), 
             _kneeLinkLength(kneeLinkLength), 
             _pHip2B(pHip2B)
{
    //LF LR RR RF
    if (legID == 0 || legID == 3)    //前腿
        _sideSign = 1;
    else if (legID == 1 || legID == 2)    //后腿
        _sideSign = -1;
    else
    {
        std::cout << "Leg ID incorrect!" << std::endl;
        exit(-1);
    }
    lastQResult.setZero();  // 初始化为零或安全位置
    hasLastResult = false;
}

// Forward Kinematics
// 计算车轮在髋关节下的坐标
template <typename T>
Vec3<T> QuadrupedLeg<T>::calcPEe2H(Vec3<T> q){
    T l1 = _hipLinkLength;
    T l2 = _kneeLinkLength;

    T s1 = std::sin(q(0));
    T s2 = std::sin(q(1));
 

    T c1 = std::cos(q(0));
    T c2 = std::cos(q(1));


    T c12 = c1 * c2 - s1 * s2;
    T c1_2 = c1 * c2 + s1 * s2;
    T s12 = s1 * c2 + c1 * s2;
    T s1_2 = s1 * c2 - c1 * s2;
    Vec3<T> pEe2H;

    pEe2H(0) = l1 * c1 + l2 * c12;  //x    x轴朝前
    pEe2H(1) =  0;  //y
    pEe2H(2) = l1 * s1 + l2 * s12;  //z
    return pEe2H;
}

// Forward Kinematics
// 计算车轮在身体坐标下的坐标
template <typename T>
Vec3<T> QuadrupedLeg<T>::calcPEe2B(Vec3<T> q){

    return _pHip2B + calcPEe2H(q);
}

// Derivative Forward Kinematics
//计算车轮末端线速度,vEe hip坐标系下 xyz
template <typename T>
Vec3<T> QuadrupedLeg<T>::calcVEe(Vec3<T> q, Vec3<T> qd){
    Vec3<T> wheel2hipVe;
    // Vec2 _p; //中间变量
    // // wheel2hipVe.topRows(2) 
    // _p = calcJaco(q) * qd.topRows(2);
    // wheel2hipVe.row(0) = _p.row(0); //x
    // wheel2hipVe.row(1).setZero();   //y方向速度为0，在hip坐标系下为0
    // wheel2hipVe.row(2) = _p.row(1);  //z
    wheel2hipVe = calcJaco(q) * qd;
    return wheel2hipVe;
}

// Inverse Kinematics
// 已知车轮位置，求解关节坐标
// Frame表示的是用哪个坐标系
template <typename T>
Vec3<T> QuadrupedLeg<T>::calcQ(Vec3<T> pEe, FrameType frame)
{
    Vec3<T> pEe2H;   //到髋关节的向量距离
    if(frame == FrameType::HIP)
        pEe2H = pEe;
    else if(frame == FrameType::BODY)
        pEe2H = pEe - _pHip2B;       //x  y z
    else{ 
        std::cout << "[ERROR] The frame of QuadrupedLeg::calcQ can only be HIP or BODY!" << std::endl;
        exit(-1);
    }
    T q1, q2,a;
    Vec3<T> qResult;
    T px, py, pz; //向量坐标

    px = pEe2H(0);  //x
    py = pEe2H(1);  //y    //定值 0 /0.2005
    pz = pEe2H(2);  //z  



    a = judge_result(px, pz);
    // std::cout << "a: " << a << std::endl;
    // 判断是否有解
    if (fabs(a)>1.00)
    {
        std::cout << "[ERROR] The length of the vector is too long!" << std::endl;
        // std::cout << "px,pz:"<< px << " " << pz << std::endl;
        return hasLastResult ? lastQResult : Vec3<T>::Zero();
        // exit(-1);
    }
    q2 = q2_ik(a);//knee
    // std::cout << "q2: " << q2 << std::endl;
    q1 = q1_ik(q2, px, pz, _hipLinkLength, _kneeLinkLength);
    // std::cout << "q1: " << q1 << std::endl;
    qResult(0) = q1;    //hip
    qResult(1) = q2;    //knee
    qResult(2) = 0;

    lastQResult = qResult;
    hasLastResult = true;
    return qResult;
}

// Derivative Inverse Kinematics
// 已知关节角度和车轮摆动速度
// 求解关节角速度
template <typename T>
Vec3<T> QuadrupedLeg<T>::calcQd(Vec3<T> q, Vec3<T> vEe,Vec3<T> qd){
    Vec3<T> qdResult;
    // qdResult.topRows(2) = calcJaco(q).inverse() * vEe.topRows(2);
    // qdResult(2) = qd(2);
    qdResult = calcJaco(q).inverse() * vEe;
    return qdResult;
}

// Derivative Inverse Kinematics
// 求解关节角速度，是上边两个函数的合体 
// vE是期望的车轮末端摆动速度，qd是当前关节角速度
template <typename T>
Vec3<T> QuadrupedLeg<T>::calcQd(Vec3<T> pEe, Vec3<T> vEe,Vec3<T> qd,FrameType frame)
{
    Vec3<T> qdresult;
    Vec3<T> q = calcQ(pEe, frame);
    // qdresult.topRows(2) = calcJaco(q).inverse() * vEe.topRows(2);
    // qdresult(2) = qd(2);
    // std::cout << "calcJaco(q).inverse(): " << calcJaco(q).inverse() << std::endl;
    qdresult = calcJaco(q).inverse() * vEe;

    return qdresult;
}

// Inverse Dynamics
// 根据关节角度计算出需要的力矩,车轮的力矩还没求解！！车轮力矩应该是包括控制速度和姿态两个方面！！！还没改完
template <typename T>
Vec3<T> QuadrupedLeg<T>::calcTau(Vec3<T> q, Vec3<T> force){
    Vec3<T> calctau;
    // Vec3<T> _force;
    // _force.row(0) = force.row(0);
    // _force.row(1) = force.row(2);
    // calctau.topRows(2) = calcJaco(q).transpose() * _force;
    // calctau.row(2).setZero();
    calctau = calcJaco(q).transpose() * force;
    return calctau;
}


template <typename T>
Mat3<T> QuadrupedLeg<T>::calcJaco(Vec3<T> q){
    Mat3<T> jaco;
    T s1 = std::sin(q(0));
    T s2 = std::sin(q(1));
    

    T c1 = std::cos(q(0));
    T c2 = std::cos(q(1));

    T c12 = c1 * c2 - s1 * s2;
    T s12 = s1 * c2 + c1 * s2;

    jaco(0, 0) = -_hipLinkLength * s1 - _kneeLinkLength * s12;
    jaco(0, 1) = - _kneeLinkLength * s12;
    jaco(0, 2) = 0;
    jaco(1, 0) = 0;
    jaco(1, 1) = 0;
    jaco(1, 2) = 1e-6;
    jaco(2, 0) = -_hipLinkLength * c1 - _kneeLinkLength * c12;
    jaco(2, 1) = -_kneeLinkLength * c12;
    jaco(2, 2) = 0;

    // jaco(0, 0) = -_hipLinkLength * s1 - _kneeLinkLength * s12;    //X方向
    // jaco(0, 1) = - _kneeLinkLength * s12;
    // jaco(0, 2) = 0;
    // jaco(1, 0) = -_hipLinkLength * c1 - _kneeLinkLength * c12;   //Z方向
    // jaco(1, 1) = -_kneeLinkLength * c12;
    // jaco(1, 2) = 0;
    // jaco(2, 0) = 1;                                            //Y方向转动
    // jaco(2, 1) = 1;
    // jaco(2, 2) = 1;

    return jaco;
}
template <typename T>
T QuadrupedLeg<T>::judge_result(T px, T pz)
{
    // T r = sqrt(px * px + pz * pz); // 计算目标点到原点的距离
    // return r <= (_hipLinkLength + _hipLinkLength) && r >= fabs(_hipLinkLength - _hipLinkLength); // 判断是否在工作空间内
    T a;  //判断是否有解
    a =(px*px+pz*pz  - _hipLinkLength*_hipLinkLength-_kneeLinkLength*_kneeLinkLength)/(2*_hipLinkLength*_kneeLinkLength);
    // a = a-0.001;
    return a;
}
template <typename T>
T QuadrupedLeg<T>::q2_ik(T gama)
{
    T q2;
    // 前腿和后腿的零点方向不同
    // 0 左前推 1 左后腿 2右后腿 3右前腿
    q2 = abs(std::acos(gama));
  
    return q2;
}
template <typename T>
T QuadrupedLeg<T>::q1_ik(T q2, T px,  T pz,T hip_l1, T knee_l2){
    T q1, k1, k2, m1;
    
    k1 = hip_l1 + knee_l2 * std::cos(q2);
    k2 = knee_l2 * std::sin(q2);
    m1 = std::atan2(k2, k1);
    q1 = (std::atan2(pz, px) - m1);
   
    return q1;
}
template class QuadrupedLeg<double>;
template class QuadrupedLeg<float>;