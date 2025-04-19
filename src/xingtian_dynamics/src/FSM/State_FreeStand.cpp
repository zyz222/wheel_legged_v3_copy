/**********************************************************************
 Copyright (c) 2020-2023, ZYZ. All rights reserved.
***********************************************************************/
#include "FSM/State_FreeStand.h"
template <typename T>
State_FreeStand<T>::State_FreeStand(CtrlComponents<T> *ctrlComp)
             :FSMState<T>(ctrlComp, FSMStateName::FREESTAND, "free stand"),
             _robModel(ctrlComp->robotModel){
    _rowMax = 50 * M_PI / 180;
    _rowMin = -_rowMax;
    _pitchMax = 50 * M_PI / 180;
    _pitchMin = -_pitchMax;
    _yawMax = 150 * M_PI / 180;
    _yawMin = -_yawMax;
    _heightMax = 0.28;   //m
    _heightMin = -0.28;    //m   世界坐标系下的位置
    
    // initRecv();
        
}
template <typename T>
void State_FreeStand<T>::enter(){
    for(int i=0; i<4; i++){
        if(this->_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO){
            this->_lowCmd->setSimFreeStanceGain(i);
            this->_lowCmd->setWheelStopGain(i);
        }
        else if(this->_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT){
            this->_lowCmd->setRealStanceGain(i);
        }
        this->_lowCmd->setZeroDq(i);
        this->_lowCmd->setZeroTau(i);
    }

    for(int i=0; i<12; i++){
        this->_lowCmd->motorCmd[i].q = this->_lowState->motorState[i].q;
    }
    _initVecOX = this->_ctrlComp->robotModel->getX(*this->_lowState);     //获取0号腿的足端位置
    _initVecXP = this->_ctrlComp->robotModel->getVecXP(*this->_lowState);  //获取其他腿相对于0号腿的足端位置，在世界坐标系下

    this->_ctrlComp->setAllStance();
    // std::cout << "State_FreeStand object at: " << this << std::endl;
    // std::cout << "_ctrlComp at: " << this->_ctrlComp << std::endl;
    // std::cout << "ioInter at: " << (this->_ctrlComp ? this->_ctrlComp->ioInter : nullptr) << std::endl;
    this->_ctrlComp->ioInter->zeroCmdPanel();     //清空用户输入指令
}
template <typename T>
void State_FreeStand<T>::run(){
    Vec34<T> vecOP;
    this->_userValue = this->_lowState->userValue;    //获取用户输入
    std::cout << "rpy:" << (this->_lowState->getRPY())*180/3.1415926 << std::endl;
    vecOP = _calcOP( invNormalize(this->_userValue.lx, _rowMin, _rowMax),     //归一化输入
                     invNormalize(this->_userValue.ly, _pitchMin, _pitchMax),
                     -invNormalize(this->_userValue.rx, _yawMin, _yawMax),
                     invNormalize(this->_userValue.ry, _heightMin, _heightMax) );

    _calcCmd(vecOP);
    // ros::spinOnce();
}
template <typename T>
void State_FreeStand<T>::exit(){
    this->_ctrlComp->ioInter->zeroCmdPanel();
}
template <typename T>
FSMStateName State_FreeStand<T>::checkChange(){
    if(this->_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else if(this->_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(this->_lowState->userCmd == UserCommand::START){
        return FSMStateName::TROTTING;
    }
    else if(this->_lowState->userCmd == UserCommand::L1_X){
        return FSMStateName::BALANCETEST;
    }
    else{
        return FSMStateName::FREESTAND;
    }
}
template <typename T>
Vec34<T> State_FreeStand<T>::_calcOP(T row, T pitch, T yaw, T height){
    Vec3<T> vecXO = -_initVecOX;           //负的0号腿的足端位置,机体坐标系下，相反表示重心在左前脚下的位置
    vecXO(2) -= height;                  //高度
    if(vecXO(2) > 0.28)
    {
        vecXO(2) = 0.28;
    }
    RotMat<T> rotM = rpyToRotMat(row, pitch, yaw);   //获取机器人的期望的旋转矩阵

    HomoMat<T> Tsb = homoMatrix(vecXO, rotM);       //机器人重心到基坐标系的变换矩阵
    HomoMat<T> Tbs = homoMatrixInverse(Tsb);

    Vec4<T> tempVec4;
    Vec34<T> vecOP;
    for(int i(0); i<4; ++i){
        tempVec4 = Tbs * homoVec(Vec3<T>(_initVecXP.col(i)));
        vecOP.col(i) = noHomoVec(tempVec4);       
    } 
    // std::cout << "最终的vecOP: " << vecOP << std::endl;
    return vecOP;     //获取到4个足端的位置
}
template <typename T>
void State_FreeStand<T>::_calcCmd(Vec34<T> vecOP){
    Vec12<T> q,dq;
    Vec34<T> WHEEL;
    q = this->_ctrlComp->robotModel->getQ(vecOP, FrameType::BODY); 
    WHEEL = this->_ctrlComp->lowState->getQ();
    q(2) = WHEEL(2,0);
    q(5) = WHEEL(2,1);
    q(8) = WHEEL(2,2);
    q(11) = WHEEL(2,3);

    this->_lowCmd->setQ(q);              //只有前两行的角度是hip和knee,而忽略了第三行wheel
}
template class State_FreeStand<double>;
// template class State_FreeStand<float>;
// void State_FreeStand::PD_Callback(const std_msgs::Float32MultiArray& msg){
//         _Kp = msg.data[0];
//         _Kd = msg.data[1];
//         _Kp_ = msg.data[2];
//         _Kd_ = msg.data[3];
    
//         for(int legID = 0; legID < 4; legID++)
//         {
//             this->_lowCmd->motorCmd[legID*3+0].mode = 1;
//             this->_lowCmd->motorCmd[legID*3+0].Kp = _Kp;
//             this->_lowCmd->motorCmd[legID*3+0].Kd = _Kd;
//             this->_lowCmd->motorCmd[legID*3+1].mode = 1;
//             this->_lowCmd->motorCmd[legID*3+1].Kp = _Kp_;
//             this->_lowCmd->motorCmd[legID*3+1].Kd = _Kd_;
//         }
        
//     }
    
// void State_FreeStand::initRecv(){    //订阅PD参数
//         _PD_Sub = _nm.subscribe("/xingtian/PD_params", 1, &State_FreeStand::PD_Callback, this);
//     }