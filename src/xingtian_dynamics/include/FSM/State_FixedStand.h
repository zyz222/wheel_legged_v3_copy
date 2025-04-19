/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef FIXEDSTAND_H
#define FIXEDSTAND_H

#include "FSM/FSMState.h"
#include "std_msgs/Float32MultiArray.h"
#include "ros/ros.h"
// template<typename T> class WBC_Ctrl;             
// template<typename T> class LocomotionCtrlData;
template <typename T>
class State_FixedStand : public FSMState<T>{
public:
    State_FixedStand(CtrlComponents<T> *ctrlComp);
    ~State_FixedStand(){}
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();
private:
         //站立状态的关节角度！！！
    T _targetPos[12] = {-2.09439, 1.347197, 0, -2.09439, 1.347197, 0, 
                            -2.09439, 1.347197, 0, -2.09439, 1.347197, 0}; 
    // 水平向前为0，逆时针为正，所有的腿都是逆时针为正。
    // T _targetPos[12] = {-1.09439, 2.347197, 0, -2.09439, 1.347197, 0, 
    //     -1.09439, 2.347197, 0, -2.09439, 1.347197, 0}; 
    T _startPos[12];
    T _duration = 1000;   //steps
    T _percent = 0;       //%
    
    Vec34<T> F;   //足端接触力
    Vec12<T> tau;    //存储12 个电机的扭矩

    Vec34<T> tau1;
    Vec34<T> q;
    Vec12<T> dq;
    Vec12<T> ddq;

    Estimator<T> *_est;
    QuadrupedRobot<T> *_robModel;

    Vec34<T> _forceFeetBody;

    T dt;
    // WBC_Ctrl<T> * _wbc_ctrl;
    // LocomotionCtrlData<T> * _wbc_data;

    // ros::Subscriber _PD_Sub;
    // ros::NodeHandle _nm;
    // void initRecv();
    // void PD_Callback(const std_msgs::Float32MultiArray &msg);
    // T _Kp, _Kd,_Kp_,_Kd_;

};

#endif  // FIXEDSTAND_H