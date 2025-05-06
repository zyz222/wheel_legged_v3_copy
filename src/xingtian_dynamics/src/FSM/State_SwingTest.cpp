/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_SwingTest.h"
template <typename T>
State_SwingTest<T>::State_SwingTest(CtrlComponents<T> *ctrlComp)
                :FSMState<T>(ctrlComp, FSMStateName::SWINGTEST, "swingTest"){
    _xMin = -150;
    _xMax =  100;
    _yMin = -150;
    _yMax =  150;
    _zMin = -50;
    _zMax =  250;
}
template <typename T>
void State_SwingTest<T>::enter(){
    for(int i=0; i<4; i++){
        if(this->_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO){
            this->_lowCmd->setSimStanceGain(i);
        }
        else if(this->_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT){
            this->_lowCmd->setRealStanceGain(i);
        }
        this->_lowCmd->setZeroDq(i);
        this->_lowCmd->setZeroTau(i);
    }
    this->_lowCmd->setSwingGain(0);

    _Kp = Vec3<T>(20, 20, 50).asDiagonal();
    _Kd = Vec3<T>( 5,  5, 20).asDiagonal();

    for(int i=0; i<12; i++){
        this->_lowCmd->motorCmd[i].q = this->_lowState->motorState[i].q;
    }

    _initFeetPos = this->_ctrlComp->robotModel->getFeet2BPositions(*this->_lowState, FrameType::HIP);
    _feetPos = _initFeetPos;     //获取足端在髋关节坐标系下的位置
    _initPos = _initFeetPos.col(0);

    this->_ctrlComp->setAllSwing();
}
template <typename T>
void State_SwingTest<T>::run(){
    this->_userValue = this->_lowState->userValue;
    //pitch
    if(this->_userValue.ly > 0){
        _posGoal(0) = invNormalize(this->_userValue.ly, _initPos(0), _initPos(0)+_xMax, 0, 1);
    }else{
        _posGoal(0) = invNormalize(this->_userValue.ly, _initPos(0)+_xMin, _initPos(0), -1, 0);
    }
    //roll
    if(this->_userValue.lx > 0){
        _posGoal(1) = invNormalize(this->_userValue.lx, _initPos(1, 0), _initPos(1)+_yMax, 0, 1);
    }else{
        _posGoal(1) = invNormalize(this->_userValue.lx, _initPos(1)+_yMin, _initPos(1), -1, 0);
    }
    //height
    if(this->_userValue.ry > 0){
        _posGoal(2) = invNormalize(this->_userValue.ry, _initPos(2), _initPos(2)+_zMax, 0, 1);
    }else{
        _posGoal(2) = invNormalize(this->_userValue.ry, _initPos(2)+_zMin, _initPos(2), -1, 0);
    }

    _positionCtrl();
    _torqueCtrl();
}
template <typename T>
void State_SwingTest<T>::exit(){
    this->_ctrlComp->ioInter->zeroCmdPanel();
}
template <typename T>
FSMStateName State_SwingTest<T>::checkChange(){
    if(this->_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(this->_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::SWINGTEST;
    }
}
template <typename T>
void State_SwingTest<T>::_positionCtrl(){
    _feetPos.col(0) = _posGoal;
    _targetPos = this->_ctrlComp->robotModel->getQ(_feetPos, FrameType::HIP);
    this->_lowCmd->setQ(_targetPos);
}
template <typename T>
void State_SwingTest<T>::_torqueCtrl(){
    Vec3<T> pos0 = this->_ctrlComp->robotModel->getFootPosition(*this->_lowState, 0, FrameType::HIP); //0号腿足端位置！！
    Vec3<T> vel0 = this->_ctrlComp->robotModel->getFootVelocity(*this->_lowState, 0);

    Vec3<T> force0 = _Kp*(_posGoal - pos0) + _Kd*(-vel0);
    // Vec2 _force0;
    // _force0(0) = force0(0);
    // _force0(1) = force0(2);
    Vec12<T> torque;
    torque.setZero();
    Mat3<T> jaco0 = this->_ctrlComp->robotModel->getJaco(*this->_lowState, 0);

    torque.segment(0, 3) = jaco0.transpose() * force0;

    this->_lowCmd->setTau(torque);
}
// template class State_SwingTest<double>;
template class State_SwingTest<float>;