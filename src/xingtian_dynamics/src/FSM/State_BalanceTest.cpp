/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_BalanceTest.h"
#include "WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp"
#include "Utilities/Utilities_print.h"
template <typename T>
State_BalanceTest<T>::State_BalanceTest(CtrlComponents<T> *ctrlComp)
                  :FSMState<T>(ctrlComp, FSMStateName::BALANCETEST, "balanceTest"),
                  _est(ctrlComp->estimator), _robModel(ctrlComp->robotModel), 
                  _balCtrl(ctrlComp->balCtrl), _contact(ctrlComp->contact){

    _xMax = 0.15;
    _xMin = -_xMax;
    _yMax = 0.05;
    _yMin = -_yMax;
    _zMax = 0.1;
    _zMin = -0.1;
    _yawMax = 20 * M_PI / 180;
    _yawMin = -_yawMax;
                    // 10  0.1  150
    _Kpp = Vec3<T>(10, 5, 150).asDiagonal();    //Kp系数
    _Kdp = Vec3<T>(0, 0.1, 0).asDiagonal();      // Kd系数

    _kpw = 150;                          //机身角速度Kp
    _Kdw = Vec3<T>(30, 30, 30).asDiagonal();    // kd

    // Turn off Foot pos command since it is set in WBC as operational task
    this->checkPDesFoot = false;                                                  //不检查足端位置是否安全

    // Initialize GRF to 0s
    this->footFeedForwardForces = Mat34<T>::Zero();                                 //初始化足端力为0，计算出来的力初始化为0

    _wbc_ctrl = new LocomotionCtrl<T>(this->_ctrlComp->_xingtian_model.buildModel());              //新建了 WBC控制器
    _wbc_data = new LocomotionCtrlData<T>();                                                              //初始化WBC数据

    _wbc_ctrl->setFloatingBaseWeight(100.);                //设置浮动基座的权重

}
template <typename T>
void State_BalanceTest<T>::enter(){
    _pcdInit = _est->cheater_getPosition();   //估计机器人的重心位置！！
    // _pcdInit = Eigen::Map<Eigen::Vector3d>(this->_lowState->xingtian_state.position);
    _pcdInit(2) = 0.25;
    _pcd = _pcdInit;
    _RdInit = this->_lowState->getRotMat();  //获取初始的旋转矩阵

    this->_ctrlComp->setAllStance();              //重置所有腿为触地状态

    _ini_body_pos = (this->_ctrlComp->estimator->cheater_getPosition()); //初始的机身位置
    // _ini_body_pos = Eigen::Map<Eigen::Vector3d>(this->_lowState->xingtian_state.position);
    if(_ini_body_pos[2] < 0.2) {
        _ini_body_pos[2] = 0.2;
      }
    last_height_command = _ini_body_pos[2];  

    _ini_body_ori_rpy = _est->getRPY();         //获取机身姿态

    _body_weight = this->_ctrlComp->_xingtian_model._bodyMass * 9.81;   

    this->_ctrlComp->ioInter->zeroCmdPanel();  //用户输入面板
    _q = vec34ToVec12(this->_lowState->getQ());
    this->_lowCmd->setChangeGain();   //设置增益，确保系统稳定
    this->_lowCmd->setQ(_q);       //设置关节角度,也就是获取当前的关节角度

    _wbc_data->pBody_des = _ini_body_pos;                  //初始化质心位置
    _wbc_data->vBody_des.setZero();                        //初始化质心速度
    _wbc_data->aBody_des.setZero();                        //初始化质心加速度

    _wbc_data->pBody_RPY_des = _ini_body_ori_rpy;                        //期望的质心欧拉角 
    for(size_t i(0); i<4; ++i){
        _wbc_data->pFoot_des[i].setZero();                                         //初始化足部位置为0
        _wbc_data->vFoot_des[i].setZero();                                         //初始化足部速度为0
        _wbc_data->aFoot_des[i].setZero();                                                //初始化足部加速度为0
        _wbc_data->Fr_des[i].setZero();                                                  //初始化足部力为0
        _wbc_data->Fr_des[i][2] =  _body_weight/4.;                                          //初始化足端力Z轴方向为
        _wbc_data->contact_state[i] = true;                                               //初始化足端接触状态为接触
        }
}
template <typename T>
void State_BalanceTest<T>::run(){

    this->_userValue = this->_lowState->userValue;  //获取用户输入值
    // _pcdInit = _est->getPosition();
    // std::cout << "pcd_init:" << _pcdInit << std::endl;
    _pcd(0) = _pcdInit(0) + invNormalize(this->_userValue.ly, _xMin, _xMax);      // 机器人重心位置 x
    _pcd(1) = _pcdInit(1) - invNormalize(this->_userValue.lx, _yMin, _yMax);     // 重心位置y
    _pcd(2) = _pcdInit(2) + invNormalize(this->_userValue.ry, _zMin, _zMax);     // 重心位置z

    BalanceStandStep();   //WBC控制部分
    for(int i = 0; i < 4; i++){
        std::cout<< "WBC tau result"<<(this->_ctrlComp->lowCmd->commands[i].tauFeedForward)<< std::endl;
        // __tau_forward.segment(3*i,3) = this->_ctrlComp->lowCmd->commands[i].tauFeedForward;
        __q.segment(3*i,3) = this->_ctrlComp->lowCmd->commands[i].qDes;
    }
    // 在世界坐标系下的重心位置
    for(int i=0; i<12; i++)
    {
     std::cout << "WBC_q: "  << __q(i) << std::endl;
    }

    T yaw = invNormalize(this->_userValue.rx, _yawMin, _yawMax);    //期望航向角,之后可以改到轮子上！！
    yaw = 0;
    _Rd = rpyToRotMat<T>(0.0, 0.0, yaw)*_RdInit;   //期望的航向角度，先将机器人绕z轴旋转yaw角度，再利用初始的旋转姿态角度
    // _Rd指的是期望与现在的机体角度的偏差！！！也就是目标姿态

    _posBody = _est->cheater_getPosition();     //获取当前机身位置,世界坐标系
    // _posBody =  Eigen::Map<Eigen::Vector3d>(this->_lowState->xingtian_state.position);

    _velBody = _est->cheater_getVelocity();     //获取当前机身速度   这个没问题
    // _velBody = Eigen::Map<Eigen::Vector3d>(this->_lowState->xingtian_state.vBody);

    
    _B2G_RotMat =this-> _lowState->getRotMat();    //获取机身到世界坐标系的旋转矩阵
    _G2B_RotMat = _B2G_RotMat.transpose();    //世界坐标系到机身的旋转矩阵



    calcTau();

    this->_lowCmd->setStableGain();   //设置增益，确保系统稳定
    this->_lowCmd->setWheelStopGain();
    this->_lowCmd->setTau(__tau_forward);     //设置关节力矩

    // this->_lowCmd->setTau(this->_ctrlComp->lowCmd->commands[i].tauFeedForward); 
    this->_lowCmd->setQ(__q);       //设置关节角度,也就是获取当前的关节角度
    // std::cout << "q: " << _q << std::endl;
}
template <typename T>
void State_BalanceTest<T>::exit(){
    this->_ctrlComp->ioInter->zeroCmdPanel();
}
//检查状态切换条件
template <typename T>
FSMStateName State_BalanceTest<T>::checkChange(){
    if(this->_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(this->_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    // else if(_lowState->userCmd == UserCommand::L2_X){
    //     return FSMStateName::FREESTAND;
    // }
    else{
        return FSMStateName::BALANCETEST;
    }
}
template <typename T>
void State_BalanceTest<T>::calcTau(){

    _ddPcd = _Kpp*(_pcd - _posBody) + _Kdp * (Vec3<T>(0, 0, 0) - _velBody);   //重心偏移需要的加速度,这个看起来没啥问题！！
    // 下边这个是姿态！这个是重心位置的差
    // std::cout << "ddPcd: " << _ddPcd << std::endl;
    // _ddPcd(0) = saturation(_ddPcd(0),Vec2<T>(-5,5));
    // _ddPcd(1) = saturation(_ddPcd(1),Vec2<T>(-5,5));
    // _ddPcd(2) = saturation(_ddPcd(2),Vec2<T>(-10,10));
    // // _ddPcd.setZero();
    //  _ddPcd(2) = 0;
    _dWbd  = _kpw*rotMatToExp<T>(_Rd*_G2B_RotMat) + _Kdw * (Vec3<T>(0, 0, 0) -this->_lowState->getGyroGlobal());  //重心偏移需要的角速度
    // _dWbd(0) = saturation(_dWbd(0),Vec2<T>(-5,5));
    // _dWbd(1) = saturation(_dWbd(1),Vec2<T>(-5,5));
    // _dWbd(2) = saturation(_dWbd(2),Vec2<T>(-1,1));
    // _dWbd =  Vec3(0, 0, 0);  //这个是姿态角度的差
    // std::cout << "dWbd: " << _dWbd << std::endl;
    _posFeet2BGlobal = _est->cheater_getPosFeet2BGlobal();   //这个获取全局坐标系下的坐标，也没问题！！！
    
    _forceFeetGlobal = _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact);
    // std::cout << "forceFeetGlobal: " << _forceFeetGlobal << std::endl;
    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;    //将足端力从全局坐标系转换到机体坐标系
 
    _q = vec34ToVec12(this->_lowState->getQ());    //获取关节角度
    for(int i = 0; i < 4; i++){
        if(_q(3*i+1)<0.0)
        _q(3*i+1) = 0.0;
        if(_q(3*i) > 0.0)
        _q(3*i) = 0.0;
    }
    _q_Dynamic = vec12ToVec34(_q);
    _dq_Dynamic = this->_lowState->getQd();   //获取当前关节角速度

    _tau = _robModel->getTau(_q, _forceFeetBody);   //静力学计算接触关节力矩
    
    _tau_forward = vec34ToVec12(_robModel->calcJointTorqueForceTotal(_q_Dynamic, _dq_Dynamic, _ddq_Dynamic, vec12ToVec34(_tau)));
}

template <typename T>
void State_BalanceTest<T>::BalanceStandStep() {      //平衡站立的核心控制，一直循环
 
                                                              //使用图形化界面控制，这里用的是这个！！！
    // Orientation
    _wbc_data->pBody_RPY_des[0] =0.6* this->_ctrlComp->_desiredStateCommand->cmdPanel->getRightStickAnalog()[0];                 //roll
    _wbc_data->pBody_RPY_des[1] = 0.6*this->_ctrlComp->_desiredStateCommand->cmdPanel->getRightStickAnalog()[1];               //pitch
    _wbc_data->pBody_RPY_des[2] -= this->_ctrlComp->_desiredStateCommand->cmdPanel->getLeftStickAnalog()[0];                      //yaw

    _wbc_data->pBody_RPY_des[0] = 0;
    _wbc_data->pBody_RPY_des[1] = 0;               //pitch
    _wbc_data->pBody_RPY_des[2] = 0;

    // Height                                                                   //高度控制
    _wbc_data->pBody_des[2] +=0.12 * this->_ctrlComp->_desiredStateCommand->cmdPanel->getHeight();
    
    _wbc_data->pBody_des[0] = _posBody(0);
    _wbc_data->pBody_des[1] = _posBody(1);
    _wbc_data->pBody_des[2] = 0.25;

    _wbc_data->vBody_Ori_des.setZero();                                          //设置质心角速度为0

    //_wbc_data->Fr_des.setZero();
    
    // std::cout << "pBody_des: " << _wbc_data->pBody_des << std::endl;  这个没问题
    // for(int i = 0; i < 4; i++)
    // {
    //     std::cout << "Fr_des: " << _wbc_data->Fr_des[i] << std::endl;
    // }
    for(size_t i(0); i<4; ++i){
        _wbc_data->pFoot_des[i] = _posFeet2BGlobal.col(i);                                         //初始化足部位置为0
        _wbc_data->vFoot_des[i].setZero();                                     //初始化足部速度为0
        _wbc_data->aFoot_des[i].setZero();                                                //初始化足部加速度为0
        _wbc_data->Fr_des[i] = _forceFeetGlobal.col(i);                                     //初始化足端力Z轴方向为
        _wbc_data->contact_state[i] = true;                                               //初始化足端接触状态为接触
        }
    
    // if(last_height_command - _wbc_data->pBody_des[2] > 0.001) {                       ///如果当前高度减去期望的机身高度大于0.001，则当前高度减去0.001  
    //     _wbc_data->pBody_des[2] = last_height_command - 0.001;
    // }
    
    last_height_command = _wbc_data->pBody_des[2];                               //每次都降低一点点！！                   

    _wbc_ctrl->run(_wbc_data, *this->_ctrlComp);                                          //运行WBC
}
// template class State_BalanceTest<double>;
template class State_BalanceTest<float>;