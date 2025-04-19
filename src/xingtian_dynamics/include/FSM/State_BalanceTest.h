/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef BALANCETEST_H
#define BALANCETEST_H

#include "FSM/FSMState.h"

template<typename T> class WBC_Ctrl;             
template<typename T> class LocomotionCtrlData;

template <typename T>
class State_BalanceTest : public FSMState<T>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    State_BalanceTest(CtrlComponents<T> *ctrlComp);
    ~State_BalanceTest(){}
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();
private:
    // Keep track of the control iterations
    int _iter = 0;
    void calcTau();
    // Parses contact specific controls to the leg controller
    void BalanceStandStep();

    WBC_Ctrl<T> * _wbc_ctrl;
    LocomotionCtrlData<T> * _wbc_data;

    Vec3<T> _ini_body_pos;
    Vec3<T> _ini_body_ori_rpy;
    T last_height_command = 0;
    T _body_weight;
    
    Estimator<T> *_est;
    QuadrupedRobot<T> *_robModel;

    BalanceCtrl<T> *_balCtrl;

    VecInt4 *_contact;

    RotMat<T> _Rd, _RdInit;      //_RdInit is the initial rotation matrix of the world frame
    Vec3<T> _pcd, _pcdInit;
    T _kpw;
    Mat3<T> _Kpp, _Kdp, _Kdw;
    Vec3<T> _ddPcd, _dWbd;

    Vec12<T> _q,_dq,_tau;
    Vec12<T> __q;
    Vec3<T> _posBody, _velBody;
    RotMat<T> _B2G_RotMat, _G2B_RotMat;
    Vec34<T> _posFeet2BGlobal;
    Vec34<T> _forceFeetGlobal, _forceFeetBody;

    T _xMax, _xMin;
    T _yMax, _yMin;
    T _zMax, _zMin;
    T _yawMax, _yawMin;

    Vec34<T> _q_Dynamic,_dq_Dynamic,_ddq_Dynamic,tau_Dynamic;  //逆动力学计算的扭矩
    Vec12<T> _tau_forward;     //最终发送的扭矩
    Vec12<T> __tau_forward;     //最终发送的扭矩   test用
};

#endif  // BALANCETEST_H