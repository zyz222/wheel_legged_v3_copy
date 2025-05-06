/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_StepTest.h"
template <typename T>
State_StepTest<T>::State_StepTest(CtrlComponents<T> *ctrlComp)
                  :FSMState<T>(ctrlComp, FSMStateName::STEPTEST, "stepTest"),
                  _est(ctrlComp->estimator), _robModel(ctrlComp->robotModel), 
                  _balCtrl(ctrlComp->balCtrl), _contact(ctrlComp->contact),
                  _phase(ctrlComp->phase){

    _gaitHeight = 50;

    _KpSwing = Vec3<T>(600, 600, 200).asDiagonal();
    _KdSwing = Vec3<T>(20, 20, 5).asDiagonal();

    _Kpp = Vec3<T>(50, 50, 300).asDiagonal();
    _Kpw = Vec3<T>(600, 600, 600).asDiagonal();
    _Kdp = Vec3<T>(5, 5, 20).asDiagonal();
    _Kdw = Vec3<T>(10, 10, 10).asDiagonal();
}
template <typename T>
void State_StepTest<T>::enter(){
    _pcd = _est->getPosition();
    _Rd  =this-> _lowState->getRotMat();
    _posFeetGlobalInit = _est->getFeetPos();
    _posFeetGlobalGoal = _posFeetGlobalInit;
    this->_ctrlComp->setStartWave();
    this->_ctrlComp->ioInter->zeroCmdPanel();
}
template <typename T>
void State_StepTest<T>::run(){
    _posBody = _est->getPosition();
    _velBody = _est->getVelocity();

    _B2G_RotMat = this->_lowState->getRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();


    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 0){
            _posFeetGlobalGoal(2, i) = _posFeetGlobalInit(2, i) + (1-cos((*_phase)(i)*2*M_PI))*_gaitHeight;
            _velFeetGlobalGoal(2, i) = sin((*_phase)(i)*2*M_PI)*2*M_PI*_gaitHeight;
        }
    }

    calcTau();

    this->_lowCmd->setZeroGain();
    this->_lowCmd->setTau(_tau);
}
template <typename T>
void State_StepTest<T>::exit(){
    this->_ctrlComp->ioInter->zeroCmdPanel();
    this->_ctrlComp->setAllSwing();
}
template <typename T>
FSMStateName State_StepTest<T>::checkChange(){
    if(this->_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(this->_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::STEPTEST;
    }
}
template <typename T>
void State_StepTest<T>::calcTau(){
    _ddPcd = _Kpp*(_pcd - _posBody) + _Kdp * (Vec3<T>(0, 0, 0) - _velBody);
    _dWbd  = _Kpw*rotMatToExp<T>(_Rd*_G2B_RotMat) + _Kdw * (Vec3<T>(0, 0, 0) - this->_lowState->getGyroGlobal());

    _posFeet2BGlobal = _est->getPosFeet2BGlobal();
    _forceFeetGlobal = - _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact);
    
    _posFeetGlobal = _est->getFeetPos();
    _velFeetGlobal = _est->getFeetVel();
    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 0){
            _forceFeetGlobal.col(i) = _KpSwing*(_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing*(_velFeetGlobalGoal.col(i)-_velFeetGlobal.col(i));
        }
    }
    
    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;

    _q = vec34ToVec12(this->_lowState->getQ());
    _tau = _robModel->getTau(_q, _forceFeetBody);
   
}
// template class State_StepTest<double>;
template class State_StepTest<float>;