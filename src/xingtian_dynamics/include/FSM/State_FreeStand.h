/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef FREESTAND_H
#define FREESTAND_H

#include "FSM/FSMState.h"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
template <typename T>
class State_FreeStand : public FSMState<T>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    State_FreeStand(CtrlComponents<T> *ctrlComp);
    ~State_FreeStand(){}
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();
private:
    Vec3<T> _initVecOX;
    Vec34<T> _initVecXP;
    T _rowMax, _rowMin;
    T _pitchMax, _pitchMin;
    T _yawMax, _yawMin;
    T _heightMax, _heightMin;

    Vec34<T> _calcOP(T row, T pitch, T yaw, T height);
    void _calcCmd(Vec34<T> vecOP);
    QuadrupedRobot<T> *_robModel;
    Vec34<T> q,dq,ddq;
    Vec34<T> tau_fw;

    // ros::Subscriber _PD_Sub;
    // ros::NodeHandle _nm;
    // void initRecv();
    // void PD_Callback(const std_msgs::Float32MultiArray &msg);
    // T _Kp, _Kd,_Kp_,_Kd_;


};

#endif  // FREESTAND_H