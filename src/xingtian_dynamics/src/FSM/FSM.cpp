/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/FSM.h"
#include <iostream>
//有限状态机，在轮腿运动中，根据输入的命令，进行状态转换
// 轮式运动，足式运动，轮腿运动
#define RESET   "\033[0m"   // 重置颜色
#define RED     "\033[31m"  // 红色
#define GREEN   "\033[32m"  // 绿色
#define YELLOW  "\033[33m"  // 黄色
#define BLUE    "\033[34m"  // 蓝色
#define MAGENTA "\033[35m"  // 品红
#define CYAN    "\033[36m"  // 青色
#define WHITE   "\033[37m"  // 白色
#define BOLD    "\033[1m"   // 加粗
template <typename T>
FSM<T>::FSM(CtrlComponents<T> *ctrlComp)
    :_ctrlComp(ctrlComp){

    _stateList.invalid = nullptr;    //无效的
    _stateList.passive = new State_Passive<T>(_ctrlComp);      //被动状态
    _stateList.fixedStand = new State_FixedStand<T>(_ctrlComp);  //固定站立 
    _stateList.freeStand = new State_FreeStand<T>(_ctrlComp);   //自由站立
    _stateList.trotting = new State_Trotting<T>(_ctrlComp);      //小跑
    _stateList.balanceTest = new State_BalanceTest<T>(_ctrlComp);   //平衡测试
    _stateList.swingTest = new State_SwingTest<T>(_ctrlComp);    //摆动测试
    _stateList.stepTest = new State_StepTest<T>(_ctrlComp);     //步态测试
    _stateList.wheelTest = new State_WheelTest<T>(_ctrlComp);    //轮式测试
    initialize();
    std::cout << "FSM initialize" << std::endl;
}
template <typename T>
FSM<T>::~FSM(){
    _stateList.deletePtr();
}
template <typename T>
void FSM<T>::initialize(){
    _currentState = _stateList.passive;    //初始状态为被动
    // std::cout << "FSM 初始模式为被动！！" << std::endl;
    _currentState -> enter();
    _nextState = _currentState;
    _mode = FSMMode::NORMAL;
}
// 主函数运行的是这个函数！！
template <typename T>
void FSM<T>::run(){
    _startTime = getSystemTime();

    _ctrlComp->sendRecv();

    _ctrlComp->runWaveGen();

    _ctrlComp->estimator->run();

    // 

    _ctrlComp->_desiredStateCommand->convertToStateCommands();    //获取期望的状态指令
    
    // std::cout << "rpy:" << (_lowState->getRPY())*360/3.1415926 << std::endl;
    if(!checkSafty()){
        _ctrlComp->ioInter->setPassive();
    }
    std::cout << "当前运动模式" << _currentState->_stateNameString << std::endl;
    if(_mode == FSMMode::NORMAL){
        // std::cout << "模式为normal 正常运行" << std::endl;
        _currentState->run();
        
        _nextStateName = _currentState->checkChange();                     //执行相应脚本中的检查状态变化函数
        // std::cout << "检查状态切换" << std::endl;
        if(_nextStateName != _currentState->_stateName){
            _mode = FSMMode::CHANGE;
            _nextState = getNextState(_nextStateName);
            std::cout << "Switched from " << _currentState->_stateNameString
                      << " to " << _nextState->_stateNameString << std::endl;
        }
    }
    else if(_mode == FSMMode::CHANGE){
        _currentState->exit();
        _currentState = _nextState;
        _currentState->enter();
        _mode = FSMMode::NORMAL;
        _currentState->run();
    }

    absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));
}
template <typename T>
FSMState<T>* FSM<T>::getNextState(FSMStateName stateName){
    switch (stateName)
    {
        
    case FSMStateName::INVALID:
        return _stateList.invalid;
        break;
    case FSMStateName::PASSIVE:
        return _stateList.passive;
        break;
    case FSMStateName::FIXEDSTAND:
        return _stateList.fixedStand;
        break;
    case FSMStateName::FREESTAND:
        return _stateList.freeStand;
        break;
    case FSMStateName::TROTTING:
        return _stateList.trotting;
        break;
    case FSMStateName::BALANCETEST:
        return _stateList.balanceTest;
        break;
    case FSMStateName::SWINGTEST:
        return _stateList.swingTest;
        break;
    case FSMStateName::STEPTEST:
        return _stateList.stepTest;
        break;
    case FSMStateName::WHEELTEST:
        return _stateList.wheelTest;
        break;
    default:
        return _stateList.invalid;
        break;
    }
}
template <typename T>
bool FSM<T>::checkSafty(){
    // The angle with z axis less than 80 degree
    if(_ctrlComp->lowState->getRotMat()(2,2) < 0.2 ){
        return false;
    }else{
        return true;
    }
}
// template class FSM<double>;
template class FSM<float>;