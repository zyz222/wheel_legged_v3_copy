/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_Passive.h"
template <typename T>
State_Passive<T>::State_Passive(CtrlComponents<T> *ctrlComp)
             :FSMState<T>(ctrlComp, FSMStateName::PASSIVE, "passive"){}

template <typename T>
void State_Passive<T>::enter(){
    if(this->_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO){
        for(int i=0; i<12; i++){

            this->_lowCmd->motorCmd[i].mode = 1;
            this->_lowCmd->motorCmd[i].q = 0;
            this->_lowCmd->motorCmd[i].dq = 0;
            this->_lowCmd->motorCmd[i].Kp = 1;
            this->_lowCmd->motorCmd[i].Kd = 0.5;
            this->_lowCmd->motorCmd[i].tau = 2;
            std::cout << "enter passive" << std::endl;
        }
    }
    else if(this->_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT){
        for(int i=0; i<12; i++){
            this->_lowCmd->motorCmd[i].mode = 1;
            this->_lowCmd->motorCmd[i].q = 0;
            this->_lowCmd->motorCmd[i].dq = 0;
            this->_lowCmd->motorCmd[i].Kp = 0;
            this->_lowCmd->motorCmd[i].Kd = 3;
            this->_lowCmd->motorCmd[i].tau = 10;
        }
    }

    this->_ctrlComp->setAllSwing();
}
template <typename T>
void State_Passive<T>::run(){
    std::cout << "State_Passive" << std::endl;
}
template <typename T>
void State_Passive<T>::exit(){

}
template <typename T>
FSMStateName State_Passive<T>::checkChange(){
    if(this->_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::PASSIVE;
    }
}
template class State_Passive<double>;
// template class State_Passive<float>;