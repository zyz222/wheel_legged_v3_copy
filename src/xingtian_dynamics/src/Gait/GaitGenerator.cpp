/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "Gait/GaitGenerator.h"
//创建摆线轨迹
template <typename T>
GaitGenerator<T>::GaitGenerator(CtrlComponents<T> *ctrlComp)
              : _waveG(ctrlComp->waveGen), _est(ctrlComp->estimator), 
                _phase(ctrlComp->phase), _contact(ctrlComp->contact), 
                _robModel(ctrlComp->robotModel), _lowState(ctrlComp->lowState){
    _feetCal = new FeetEndCal<T>(ctrlComp);
    _firstRun = true;
}
template <typename T>
GaitGenerator<T>::~GaitGenerator(){
}
//传进来的是机身坐标下的位置和速度。在此基础上进行求解
template <typename T>
void GaitGenerator<T>::setGait(Vec2<T> vxyGoalGlobal, T dYawGoal, T gaitHeight){     //
    _vxyGoal = vxyGoalGlobal;      //机身坐标下的速度
    _dYawGoal = dYawGoal;     
    _gaitHeight = gaitHeight;
}
template <typename T>
void GaitGenerator<T>::restart(){
    _firstRun = true;
    _vxyGoal.setZero();
}
//将其全部改为机身坐标系下运动！！！！
template <typename T>
void GaitGenerator<T>::run(Vec34<T> &feetPos, Vec34<T> &feetVel){
    if(_firstRun){
        // _startP = _est->cheater_getFeetPos();     //获取足端位置，世界坐标系下
        _startP = _robModel->getFeet2BPositions(*_lowState,FrameType::BODY);

        _firstRun = false;
    }

    
    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 1){
            if((*_phase)(i) < 0.5){                   //stance
                // _startP.col(i) = _est->cheater_getFootPos(i);    //获取开始时的足端位置,站立相位
                _startP.col(i) = _robModel->getFootPosition(*_lowState,i,FrameType::BODY);//获取开始时的足端位置,站立相位.机身坐标系下 XYZ
                // feetVel.col(i) = _robModel->getFootVelocity(*_lowState,i);    //这个相对于髋关节的速度，也可是说是相对于机身坐标系 XYZ
                // _startP.col(i)(2) = -0.22;
            }
            feetPos.col(i) = _startP.col(i);   //这个也没问题
            // std::cout << "_startP: " << _startP.col(i) << std::endl;
            feetVel.col(i).setZero();
        }
        else{
            // std::cout << "vxy: " << _vxyGoal << std::endl;
            _endP.col(i) = _feetCal->calFootPos(i, _vxyGoal, _dYawGoal, (*_phase)(i));  //计算足端位置,机身坐标系下的足端位置 XYZ
            // std::cout << "endP: " << _endP.col(i) << std::endl;

            feetPos.col(i) = getFootPos(i);
            feetVel.col(i) = getFootVel(i);
        }
    }
    // _pastP = feetPos;
    _phasePast = *_phase;
}
template <typename T>
Vec3<T> GaitGenerator<T>::getFootPos(int i){
    Vec3<T> footPos;
    // std::cout << "startP: " << _startP.col(i) << std::endl;
    // std::cout << "endP: " << _endP.col(i) << std::endl;
    //  _startP.col(i)(1) = _endP.col(i)(1) ;
    footPos(0) = cycloidXYPosition(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    footPos(1) = cycloidXYPosition(_startP.col(i)(1), _endP.col(i)(1), (*_phase)(i));
    // footPos(1) = _startP.col(i)(1);
    footPos(2) =  cycloidZPosition(_startP.col(i)(2), _gaitHeight, (*_phase)(i));
    
    // std::cout << "footPos: " << footPos << std::endl;
    return footPos;
}
template <typename T>
Vec3<T> GaitGenerator<T>::getFootVel(int i){
    Vec3<T> footVel;

    footVel(0) = cycloidXYVelocity(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    footVel(1) = cycloidXYVelocity(_startP.col(i)(1), _endP.col(i)(1), (*_phase)(i));
    // footVel(1) = _est->getFeetVel().col(i)(1);
    footVel(2) =  cycloidZVelocity(_gaitHeight, (*_phase)(i));
    // std::cout << "footVel: " << footVel << std::endl;

    return footVel;
}
template <typename T>
T GaitGenerator<T>::cycloidXYPosition(T start, T end, T phase){
    T phasePI = 2 * M_PI * phase;
    return (end - start)*(phasePI - sin(phasePI))/(2*M_PI) + start;
}
template <typename T>
T GaitGenerator<T>::cycloidXYVelocity(T start, T end, T phase){
    T phasePI = 2 * M_PI * phase;
    return (end - start)*(1 - cos(phasePI)) / _waveG->getTswing();
}
template <typename T>
T GaitGenerator<T>::cycloidZPosition(T start, T h, T phase){
    T phasePI = 2 * M_PI * phase;
    return h*(1 - cos(phasePI))/2 + start;
}
template <typename T>
T GaitGenerator<T>::cycloidZVelocity(T h, T phase){
    T phasePI = 2 * M_PI * phase;
    return h*M_PI * sin(phasePI) / _waveG->getTswing();
}
// template class GaitGenerator<double>;
template class GaitGenerator<float>;