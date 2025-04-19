/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef STEPTEST_H
#define STEPTEST_H

#include "FSM/FSMState.h"

template<typename T> class CtrlComponents;
template<typename T> class LowlevelState;
template <typename T>
class State_StepTest : public FSMState<T>{
public:
    State_StepTest(CtrlComponents<T> *ctrlComp);
    ~State_StepTest(){}
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();
private:
    void calcTau();

    T _gaitHeight;

    Estimator<T> *_est;
    QuadrupedRobot<T> *_robModel;
    BalanceCtrl<T> *_balCtrl;

    VecInt4 *_contact;
    Vec4<T> *_phase;

    RotMat<T> _Rd;
    Vec3<T> _pcd;
    Mat3<T> _Kpp, _Kpw, _Kdp, _Kdw;
    Mat3<T> _KpSwing, _KdSwing;
    Vec3<T> _ddPcd, _dWbd;

    Vec12<T> _q, _tau;
    Vec3<T> _posBody, _velBody;
    RotMat<T> _B2G_RotMat, _G2B_RotMat;
    Vec34<T> _posFeet2BGlobal;
    Vec34<T> _posFeetGlobalInit, _posFeetGlobalGoal, _velFeetGlobalGoal;
    Vec34<T> _posFeetGlobal, _velFeetGlobal;
    Vec34<T> _forceFeetGlobal, _forceFeetBody;
};

#endif  // STEPTEST_H