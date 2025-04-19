/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/FSMState.h"
//获取机器人的底层状态，包括控制指令和机器人状态！！！
template <typename T>
FSMState<T>::FSMState(CtrlComponents<T> *ctrlComp, FSMStateName stateName, std::string stateNameString)
            :_ctrlComp(ctrlComp), _stateName(stateName), _stateNameString(stateNameString){
    _lowCmd = _ctrlComp->lowCmd;
    _lowState = _ctrlComp->lowState;
}
template class FSMState<double>;
// template class FSMState<float>;


