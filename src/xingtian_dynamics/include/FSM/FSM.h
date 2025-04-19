/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef FSM_H
#define FSM_H

// FSM States
#include "FSM/FSMState.h"
#include "FSM/State_FixedStand.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FreeStand.h"
#include "FSM/State_Trotting.h"
#include "FSM/State_BalanceTest.h"
#include "FSM/State_SwingTest.h"
#include "FSM/State_StepTest.h"
#include "FSM/State_WheelTest.h"
#include "common/enumClass.h"
#include "control/CtrlComponents.h"

template <typename T>
struct FSMStateList{
    FSMState<T> *invalid;
    State_Passive<T> *passive;
    State_FixedStand<T> *fixedStand;
    State_FreeStand<T> *freeStand;
    State_Trotting<T> *trotting;
    State_BalanceTest<T> *balanceTest;
    State_SwingTest<T> *swingTest;
    State_StepTest<T> *stepTest;
    State_WheelTest<T> *wheelTest;
    
    void deletePtr(){
        delete invalid;
        delete passive;
        delete fixedStand;
        delete freeStand;
        delete trotting;
        delete balanceTest;
        delete swingTest;
        delete stepTest;
        delete wheelTest;
    }
};
template <typename T>
class FSM{
public:
    FSM(CtrlComponents<T> *ctrlComp);
    ~FSM();
    void initialize();
    void run();
private:
    FSMState<T>* getNextState(FSMStateName stateName);
    bool checkSafty();
    CtrlComponents<T> *_ctrlComp;
    FSMState<T> *_currentState;    //当前状态
    FSMState<T> *_nextState;
    FSMStateName _nextStateName;
    FSMStateList<T> _stateList;
    FSMMode _mode;

    LowlevelState<T> *_lowState;
    long long _startTime;
    int count;
};


#endif  // FSM_H