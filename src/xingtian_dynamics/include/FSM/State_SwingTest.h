/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef STATE_SWINGTEST_H
#define STATE_SWINGTEST_H

#include "FSM/FSMState.h"
#include "Gait/GaitGenerator.h"
template <typename T>
class State_SwingTest : public FSMState<T>{
public:
    State_SwingTest(CtrlComponents<T> *ctrlComp);
    ~State_SwingTest(){};
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();
private:
    void _positionCtrl();
    void _torqueCtrl();

    Vec34<T> _initFeetPos, _feetPos;
    Vec3<T>  _initPos, _posGoal;
    Vec12<T> _targetPos;
    T _xMin, _xMax;
    T _yMin, _yMax;
    T _zMin, _zMax;
    Mat3<T> _Kp, _Kd;
};

#endif  // STATE_SWINGTEST_H