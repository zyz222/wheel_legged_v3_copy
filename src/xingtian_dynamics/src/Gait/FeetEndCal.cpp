/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "Gait/FeetEndCal.h"
#include "common/mathTools.h"
// 该类的主要功能是根据机器人的当前状态（如速度、角速度、姿态等）和目标运动参数
// （如期望的速度和偏航角变化），计算每个足端在全局坐标系中的位置。
template <typename T>
FeetEndCal<T>::FeetEndCal(CtrlComponents<T> *ctrlComp)
           : _est(ctrlComp->estimator), _lowState(ctrlComp->lowState),
             _robModel(ctrlComp->robotModel){
    _Tstance  = ctrlComp->waveGen->getTstance();   //获取站立相位时间
    _Tswing   = ctrlComp->waveGen->getTswing();   //获取摆动相位时间

    // _kx = 0.15;   //用于调节足端x位置的比例增益
    _kx = 0.15;
    _ky = 0.005;
    _kyaw = 0.005;

    Vec34<T> feetPosBody = _robModel->getFeetPosIdeal();       //获取机器人足部位置，中性足端位置，机体坐标系下
    for(int i(0); i<4; ++i){
        _feetRadius(i) = sqrt(pow(feetPosBody(0, i), 2) + pow(feetPosBody(1, i), 2) );   //计算第i个足端到机器人重心的水平距离 x y
        _feetInitAngle(i) = atan2(feetPosBody(1, i), feetPosBody(0, i));   //计算第i个足端相对于机体坐标系x轴的角度
        // _feetInitAngle(i) = 0; 
    }
}
template <typename T>
FeetEndCal<T>::~FeetEndCal(){

}
//，根据机体速度，航向角度，和当前足端位置，计算足端坐标，phase指的是相位，从站立相开始最后回到站立相位。0.5        这个函数有问题，等待修改！！！
//_Tswing 摆动相时间   变量名称没改！！！
template <typename T>
Vec3<T> FeetEndCal<T>::calFootPos(int legID, Vec2<T> vxyGoalGlobal, T dYawGoal, T phase){

    _bodyVelGlobal = _lowState->getRotMat().transpose()*_est->cheater_getVelocity();     //获取机身当前速度，世界坐标下

    //  = _est->cheater_getFootPos(legID);
    // _bodyWGlobal = this->_lowState->getGyro();   //获取机身当前角速度，机身坐标下
    //存储下一步的位置
    _nextStep(0) = _bodyVelGlobal(0)*(1-phase)*_Tswing + _bodyVelGlobal(0)*_Tstance/2 + _kx*(vxyGoalGlobal(0)-_bodyVelGlobal(0));
    // _nextStep(1) = _bodyVelGlobal(1)*(1-phase)*_Tswing + _bodyVelGlobal(1)*_Tstance/2 + _ky*(vxyGoalGlobal(1)-_bodyVelGlobal(1)); //机身y轴速度，世界坐标系下
    _nextStep(1) = 0;
    // _nextStep(2) = -0.22; 
    // _nextStep(2) = _robModel->getFootPosition(*(this->_lowState),legID,FrameType::BODY)(2);   
    _nextStep(2) = 0;                  //有改进的空间！！
    // _yaw = _lowState->getYaw();
    // // _yaw = 0;
    // _dYaw = _lowState->getDYaw();
    // _nextYaw = _dYaw*(1-phase)*_Tswing + _dYaw*_Tstance/2 + _kyaw*(dYawGoal - _dYaw);

    _nextStep(0) = saturation(_nextStep(0), Vec2<T>(-0.1,0.1));
    
    // // 从此从此
    // // _footPos = _est->getPosition() + _nextStep;      //机身当前位置，全局坐标下，往前加了X方向距离
    // // std::cout<<"_footPos: "<<_footPos<<std::endl;
    // _nextStep(0) += _feetRadius(legID) * cos(_yaw + _feetInitAngle(legID) + _nextYaw);    //转换到各个腿上
    // _nextStep(1) += _feetRadius(legID) * sin(_yaw + _feetInitAngle(legID) + _nextYaw);

    // _nextStep(0) = saturation(_nextStep(0), Vec2<T>(_est->cheater_getFootPos(legID)(0)-0.05,_est->cheater_getFootPos(legID)(0)+0.05));    //限制它每次只能走0.15m
    // _nextStep(1) = saturation(_nextStep(1), Vec2<T>(_est->cheater_getFootPos(legID)(1)-0.05,_est->cheater_getFootPos(legID)(1)+0.05));
    // // std::cout<<"_nextStep(1): "<<_nextStep(1)<<std::endl;
    // // _footPos = _est->getPosition() + _nextStep;
    // // _footPos = _nextStep;
    _footPos = _robModel->getFootPosition(*_lowState,legID,FrameType::BODY) +_nextStep;
    // _footPos = _est->cheater_getFootPos(legID) + _nextStep;
    _footPos(2) = 0.0;
    // std::cout<<"_footPos: "<<_footPos<<std::endl;
    return _footPos;
}
template class FeetEndCal<double>;
// template class FeetEndCal<float>;