/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "control/ControlFrame.h"
template <typename T>
ControlFrame<T>::ControlFrame(CtrlComponents<T> *ctrlComp):_ctrlComp(ctrlComp){
    _FSMController = new FSM<T>(_ctrlComp);
}
template <typename T>
void ControlFrame<T>::run(){
    _FSMController->run();
}
template class ControlFrame<double>;
// template class ControlFrame<float>;