/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#ifndef C08BFA56_85C9_45F0_890A_7475E775FB19
#define C08BFA56_85C9_45F0_890A_7475E775FB19
#ifndef TROTTING_H
#define TROTTING_H

#include "FSM/FSMState.h"
#include "Gait/GaitGenerator.h"
#include "control/BalanceCtrl.h"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
template <typename T>
class State_Trotting : public FSMState<T>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    State_Trotting(CtrlComponents<T> *ctrlComp);
    ~State_Trotting();
    void enter();
    void run();
    void exit();
    virtual FSMStateName checkChange();
    void setHighCmd(T vx, T vy, T wz);
private:
    void calcTau();
    void calcQQd();
    void calcCmd();
    virtual void getUserCmd();
    void calcBalanceKp();
    bool checkStepOrNot();

    GaitGenerator<T> *_gait;
    Estimator<T> *_est;
    QuadrupedRobot<T> *_robModel;
    BalanceCtrl<T> *_balCtrl;

    // Rob State
    Vec3<T>  _posBody, _velBody;
    T _yaw, _dYaw;
    Vec34<T> _posFeetGlobal, _velFeetGlobal;
    Vec34<T> _posFeet2BGlobal;
    RotMat<T> _B2G_RotMat, _G2B_RotMat;
    Vec12<T> _q,_qd;

    // Robot command
    Vec3<T> _pcd;
    Vec3<T> _vCmdGlobal, _vCmdBody;
    T _yawCmd, _dYawCmd;
    T _dYawCmdPast;
    Vec3<T> _wCmdGlobal,_wCmdBody;
    Vec34<T> _posFeetGlobalGoal, _velFeetGlobalGoal;
    Vec34<T> _posFeet2BGoal, _velFeet2BGoal;
    RotMat<T> _Rd;
    Vec3<T> _ddPcd, _dWbd;
    Vec34<T> _forceFeetGlobal, _forceFeetBody;
    Vec34<T> _qGoal, _qdGoal,_qddGoal;
    Vec12<T> _tau,_tau_forward;

    // Control Parameters
    T _gaitHeight;
    Vec3<T> _posError, _velError;
    Mat3<T> _Kpp, _Kdp, _Kdw;
    T _kpw;
    Mat3<T> _KpSwing, _KdSwing;
    Vec2<T> _vxLim, _vyLim, _wyawLim;
    Vec4<T> *_phase;
    Vec4<T> *_contact;

    ros::Subscriber _PD_Sub;
    ros::NodeHandle _nm;
    void initRecv();
    void PD_Callback(const std_msgs::Float32MultiArray &msg);
    T _Kp, _Kd,_Kp_,_Kd_;
    // Calculate average value
    AvgCov<T> *_avg_posError = new AvgCov<T>(3, "_posError", true, 1000, 1000, 1);
    AvgCov<T> *_avg_angError = new AvgCov<T>(3, "_angError", true, 1000, 1000, 1000);
};

#endif  // TROTTING_H


#endif /* C08BFA56_85C9_45F0_890A_7475E775FB19 */
