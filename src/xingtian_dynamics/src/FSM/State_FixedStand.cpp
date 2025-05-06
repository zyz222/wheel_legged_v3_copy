/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include <iostream>
#include "FSM/State_FixedStand.h"
#include "common/cppTypes.h"
#include "WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp"
#define RESET   "\033[0m"   // 重置颜色
#define RED     "\033[31m"  // 红色
#define GREEN   "\033[32m"  // 绿色
#define YELLOW  "\033[33m"  // 黄色
#define BLUE    "\033[34m"  // 蓝色
#define MAGENTA "\033[35m"  // 品红
#define CYAN    "\033[36m"  // 青色
#define WHITE   "\033[37m"  // 白色
#define BOLD    "\033[1m"   // 加粗
//所有y轴朝右，遵从右手定则。这个是位置控制环
template <typename T>
State_FixedStand<T>::State_FixedStand(CtrlComponents<T> *ctrlComp)
                :FSMState<T>(ctrlComp, FSMStateName::FIXEDSTAND, "fixed stand"),
                _robModel(ctrlComp->robotModel){
                // initRecv();
                // _wbc_ctrl = new LocomotionCtrl<T>(this->_ctrlComp->_xingtian_model.buildModel());              //新建了 WBC控制器
                // _wbc_data = new LocomotionCtrlData<T>();                                                              //初始化WBC数据

                // _wbc_ctrl->setFloatingBaseWeight(100.);   
                }
template <typename T>
void State_FixedStand<T>::enter(){
    //    这里是原来的位置控制代码
    // for(int i=0; i<4; i++){
    //     if(this->_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO){
    //           //位置控制
    //         this->_lowCmd->setSwingGain(i);
    //         this->_lowCmd->setWheelStopGain(i);
    //     }
    //     else if(this->_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT){
    //         this->_lowCmd->setRealStanceGain(i);
    //     }
    //     this->_lowCmd->setZeroDq(i);    //角速度设为0
    //     this->_lowCmd->setZeroTau(i);   //力矩设置为0
    // }  
    for(int i=0; i<12; i++){
        this->_lowCmd->motorCmd[i].q = this->_lowState->motorState[i].q;
        _startPos[i] = this->_lowState->motorState[i].q;
    }
    this->_ctrlComp->setAllStance();

    // // // //这里是修改后的

    for(int i = 0; i < 4; i++){
        this->_lowCmd->setDynamicsGain(i);
        this->_lowCmd->setWheelStopGain(i);
        this->_lowCmd->setZeroTau(i);
        this->_lowCmd->setZeroDq(i);

    }
    F.setZero();
    _forceFeetBody(0,0) = 0;
    _forceFeetBody(0,1) = -0;
    _forceFeetBody(0,2) = -0;
    _forceFeetBody(0,3) = 0;
    _forceFeetBody(2,0) = 80;
    _forceFeetBody(2,1) = 80;
    _forceFeetBody(2,2) = 80;
    _forceFeetBody(2,3) = 80;
    ddq.setZero();
    dq.setZero();
//     // dt = 0.1;
}
//通过线性插值，控制每个机器人的关节位置。
// 这里也是需要修改的！
template <typename T>
void State_FixedStand<T>::run(){
    _percent += (T)1/_duration;    //从初始位置到目标位置的过渡进度。
    _percent = _percent > 1 ? 1 : _percent;
    // std::cout<<"RPY"<< this->_lowState->getRPY()<<endl;
    // std::cout<<"V"<< this->_lowState->getGyro()<<endl;
    //   这是原来的
    for(int j=0; j<12; j++){
        this->_lowCmd->motorCmd[j].q = (1 - _percent)*_startPos[j] + _percent*_targetPos[j]; 
        this->_lowCmd->motorCmd[j].dq = (_targetPos[j] - _startPos[j])/_duration;
        ddq[j] = (_targetPos[j] - _startPos[j])/_duration;
        if (j == 2 or j == 5 or j == 8 or j == 11 )
        {
            this->_lowCmd->motorCmd[j].q = _startPos[j];
            this->_lowCmd->motorCmd[j].dq = 0;
            ddq[j] = 0;
        } 
        if(_percent == 1)
        {
            this->_lowCmd->motorCmd[j].dq = 0;
            ddq[j] = 0;
        }
    }

    q << this->_lowState->motorState[0].q, this->_lowState->motorState[3].q, this->_lowState->motorState[6].q, this->_lowState->motorState[9].q,
    this->_lowState->motorState[1].q, this->_lowState->motorState[4].q, this->_lowState->motorState[7].q, this->_lowState->motorState[10].q,
    this->_lowState->motorState[2].q, this->_lowState->motorState[5].q, this->_lowState->motorState[8].q, this->_lowState->motorState[11].q;
          
    dq << this->_lowState->motorState[0].dq, this->_lowState->motorState[3].dq, this->_lowState->motorState[6].dq, this->_lowState->motorState[9].dq,
    this->_lowState->motorState[1].dq, this->_lowState->motorState[4].dq, this->_lowState->motorState[7].dq, this->_lowState->motorState[10].dq,
    this->_lowState->motorState[2].dq, this->_lowState->motorState[5].dq, this->_lowState->motorState[8].dq, this->_lowState->motorState[11].dq;



    F = vec12ToVec34(_robModel->getTau(vec34ToVec12(q), _forceFeetBody));

    Vec34<T> _dq = vec12ToVec34(dq);
    Vec34<T> _ddq = vec12ToVec34(ddq);
    tau1 = _robModel->calcJointTorqueForceTotal(q,_dq,_ddq,F);

    Vec12<T> tau1_vector = vec34ToVec12(tau1);
    // 设置扭矩
    std::cout<<"tau1"<<tau1<<endl;
    this->_lowCmd->setTau(tau1_vector);
    // ros::spinOnce();
    // this->_ctrlComp->robotModel->forwardKinematics();
    // _robModel->forwardKinematics();
}
template <typename T>
void State_FixedStand<T>::exit(){
    _percent = 0;
}
template <typename T>
FSMStateName State_FixedStand<T>::checkChange(){
    if(this->_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(this->_lowState->userCmd == UserCommand::L2_X){
        return FSMStateName::FREESTAND;
    }
    else if(this->_lowState->userCmd == UserCommand::START){
        return FSMStateName::TROTTING;
    }
    else if(this->_lowState->userCmd == UserCommand::L1_X){
        return FSMStateName::BALANCETEST;
    }
    else if(this->_lowState->userCmd == UserCommand::L1_A){
        return FSMStateName::SWINGTEST;
    }
    else if(this->_lowState->userCmd == UserCommand::L1_Y){
        return FSMStateName::STEPTEST;
    }
    else if(this->_lowState->userCmd == UserCommand::ROLL){
        return FSMStateName::WHEELTEST;
    }
    else{
        return FSMStateName::FIXEDSTAND;
    }
}
// template class State_FixedStand<double>;
template class State_FixedStand<float>;


// void State_FixedStand::PD_Callback(const std_msgs::Float32MultiArray& msg){
//     _Kp = msg.data[0];
//     _Kd = msg.data[1];
//     _Kp_ = msg.data[2];
//     _Kd_ = msg.data[3];

//     for(int legID = 0; legID < 4; legID++)
//     {
//         this->_lowCmd->motorCmd[legID*3+0].mode = 1;
//         this->_lowCmd->motorCmd[legID*3+0].Kp = _Kp;
//         this->_lowCmd->motorCmd[legID*3+0].Kd = _Kd;
//         this->_lowCmd->motorCmd[legID*3+1].mode = 1;
//         this->_lowCmd->motorCmd[legID*3+1].Kp = _Kp_;
//         this->_lowCmd->motorCmd[legID*3+1].Kd = _Kd_;
//     }
    
// }

// void State_FixedStand::initRecv(){    //订阅速度指令
//     _PD_Sub = _nm.subscribe("/xingtian/PD_params", 1, &State_FixedStand::PD_Callback, this);
// }