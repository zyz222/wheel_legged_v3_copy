/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef PASSIVE_H
#define PASSIVE_H

#include "FSMState.h"
template <typename T>
class State_Passive : public FSMState<T>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    State_Passive(CtrlComponents<T> *ctrlComp);
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();
};

#endif  // PASSIVE_H