/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_Trotting.h"
#include <iomanip>
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
State_Trotting<T>::State_Trotting(CtrlComponents<T> *ctrlComp)
             :FSMState<T>(ctrlComp, FSMStateName::TROTTING, "trotting"), 
              _est(ctrlComp->estimator), _phase(ctrlComp->phase), 
              _contact(ctrlComp->contact), _robModel(ctrlComp->robotModel), 
              _balCtrl(ctrlComp->balCtrl){
                initRecv();
    
    _gait = new GaitGenerator<T>(ctrlComp);  //还没修改完

    _gaitHeight = 0.02;       //世界坐标系下的步态高度

    // _Kpp = Vec3(500, 0.1, 500).asDiagonal();     //用于机身平衡控制器
    _Kpp =Vec3<T>(120, 50, 180).asDiagonal();

    _Kdp = Vec3<T>(10, 0.1, 1.2).asDiagonal();
    _kpw = 150; 
    _Kdw = Vec3<T>(10, 10, 0).asDiagonal();
    _KpSwing = Vec3<T>(120, 0.1, 120).asDiagonal();   //摆动腿足端修正力参数
    _KdSwing = Vec3<T>(1.2, 0.1, 1.2).asDiagonal();


    _vxLim = _robModel->getRobVelLimitX();     //获取机器人的x速度限制
    _vyLim = _robModel->getRobVelLimitY();       //获取机器人的y速度限制
    _wyawLim = _robModel->getRobVelLimitYaw();   //获取机器人的yaw速度限制

}
template <typename T>
State_Trotting<T>::~State_Trotting(){
    delete _gait;
}
template <typename T>
void State_Trotting<T>::enter(){
    // std::cout << "State_Trotting::enter()" << std::endl;
    _pcd = _est->cheater_getPosition();
    
    _pcd(2) = -_robModel->getFeetPosIdeal()(2, 0);   //中性落足点位置，世界坐标系下左前腿的高！！
    // std::cout << "State_Trotting:_pcb:" << _pcd<< std::endl;
    _vCmdBody.setZero();
    _yawCmd = this->_lowState->getYaw();
    _Rd = rotz(_yawCmd);
    _wCmdGlobal.setZero();

    this->_ctrlComp->ioInter->zeroCmdPanel();
    _gait->restart();
}
template <typename T>
void State_Trotting<T>::exit(){
    this->_ctrlComp->ioInter->zeroCmdPanel();
    this->_ctrlComp->setAllSwing();
}


template <typename T>
void State_Trotting<T>::run(){

    _posBody = _est->cheater_getPosition();   //获取机身世界坐标系下的位置
    // std::cout << "_posbody" << _posBody << std::endl;
    _velBody = _est->cheater_getVelocity();   //获取机身速度
    // std::cout << "_velbody" << _velBody << std::endl;
    _posFeet2BGlobal = _est->cheater_getPosFeet2BGlobal();  //获取足部在相对于机体的位置世界坐标系下,
    // std::cout << "posFeet2BGlobal: " << _posFeet2BGlobal << std::endl;
    _posFeetGlobal  = _est->cheater_getFeetPos();          //获取足部在世界坐标系下的位置
    _velFeetGlobal = _est->cheater_getFeetVel();           //获取足部在世界坐标系下的速度

    _posFeet2BGoal = _robModel->getFeet2BPositions(*this->_lowState,FrameType::BODY);  //获取机身坐标系下的足端位置
    _velFeet2BGoal = _robModel->getFeet2BVelocities(*this->_lowState,FrameType::BODY);  //获取机身坐标下的的足端速度

    _B2G_RotMat = this->_lowState->getRotMat();         //获取机体到世界坐标系的旋转矩阵，这个不可能有问题！！
    _G2B_RotMat = _B2G_RotMat.transpose();          //获取世界坐标系到机体的旋转矩阵
    _yaw = this->_lowState->getYaw();                    //获取机身yaw角
    _dYaw = this->_lowState->getDYaw();                   //获取机身yaw角速度

    this->_userValue = this->_lowState->userValue;

    getUserCmd();

    calcCmd();
    //步态高度，这里是个定值  魔改下边这两个函数！！

    
    //这里是魔改之后的
    // _gait->setGait(_vCmdGlobal.segment(0,2), _wCmdGlobal(2), _gaitHeight);
    _gait->setGait(_vCmdBody.segment(0,2), _wCmdBody(2), _gaitHeight);
    _gait->run(_posFeet2BGoal, _velFeet2BGoal);   //这两个是机身坐标下的位置和速度
    // _gait->run(_posFeetGlobalGoal, _velFeetGlobalGoal); 
    calcTau();
    calcQQd();   //计算关节角速度

    if(checkStepOrNot()){
        this->_ctrlComp->setStartWave();
    }else{
        this->_ctrlComp->setAllStance();
    }


   

    this->_lowCmd->setTau(_tau_forward);    //满足扭矩的限制
    // std::cout << BLUE << "tau: " << _tau  << std::endl;
    this->_lowCmd->setQ(vec34ToVec12(_qGoal));
  
    // std::cout << BLUE << "_qGoal: " << _qGoal  << std::endl;
    this->_lowCmd->setQd(vec34ToVec12(_qdGoal));

    // std::cout << BLUE << "_qdGoal: " << _qdGoal <<RESET << std::endl;
    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 0){
            this->_lowCmd->setSwingGain(i);
            this->_lowCmd->setWheelStopGain(i);
        std::cout << RED << "摆动腿" << RESET << std::endl;
        }else{
            this->_lowCmd->setStableTrotGain(i);
            this->_lowCmd->setWheelStopGain(i);
            std::cout << YELLOW << "站立腿" << RESET << std::endl;
        }
    }
}


template <typename T>
void State_Trotting<T>::getUserCmd(){
    /* Movement */
    _vCmdBody(0) =  invNormalize(this->_userValue.ly, _vxLim(0), _vxLim(1));   //x方向速度，归一化到-1，1
    // _vCmdBody(1) = -invNormalize(this->_userValue.lx, _vyLim(0), _vyLim(1));    //y方向速度
    _vCmdBody(1) = 0;
    _vCmdBody(2) = 0;
    _wCmdBody.setZero();

    // _dYawCmd = 0.0;
    _dYawCmd = -invNormalize(this->_userValue.rx, _wyawLim(0), _wyawLim(1));
    _dYawCmd = 0.9*_dYawCmdPast + (1-0.9) * _dYawCmd;
    _dYawCmdPast = _dYawCmd;
    // std::cout << "vx: " << _vCmdBody(0) << " vy: " << _vCmdBody(1) << " wyaw: " << _dYawCmd << std::endl;
}
//计算目标速度和偏航角
template <typename T>
void State_Trotting<T>::calcCmd(){    
    /* Movement */
    _vCmdGlobal = _B2G_RotMat * _vCmdBody;

     //pcd 目标位置,世界坐标系下重心位置，最大偏差限制在正负0.05m
    // if(_vCmdBody(0) != 0 || _vCmdBody(1) != 0)
    // {
        _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2<T>(_velBody(0)-0.2, _velBody(0)+0.2));  //机身x方向速度
        _vCmdGlobal(1) = saturation(_vCmdGlobal(1), Vec2<T>(_velBody(1)-0.2, _velBody(1)+0.2));
        _vCmdGlobal(2) = 0;
        _pcd(0) = saturation(_pcd(0) + _vCmdGlobal(0) * this->_ctrlComp->dt, _posBody(0) - 0.25, _posBody(0) + 0.25);
        // _pcd(1) = saturation(_pcd(1) + _vCmdGlobal(1) * this->_ctrlComp->dt, _posBody(1) - 0.05, _posBody(1) + 0.05);
        // _pcd(2) = saturation(_pcd(2) + _vCmdGlobal(2) * this->_ctrlComp->dt, _posBody(2) - 0.05, _posBody(2) + 0.05);
    // }

    
    _Rd = rpyToRotMat<T>(0, 0, _yawCmd)*_Rd;
    // _Rd = rotz(_yawCmd);
    _wCmdGlobal(2) = _dYawCmd;
    // std::cout << "yawCmd:" << _yawCmd << std::endl;
}
template <typename T>
void State_Trotting<T>::calcTau(){      //站立相的控制方法

    _posError = _pcd - _posBody;        //世界坐标系下的机身位置误差
    _velError = _vCmdGlobal - _velBody;  //机身速度误差
    std::cout << YELLOW << "posError: " << _posError << std::endl;
    std::cout << RED << "velError: " << _velError << std::endl;
    // if(abs(_posError(0)) < 0.1)
    //   {  _posError(0) = 0;}
    _posError(1) = 0;
    _posError(0) = 0;
    _ddPcd = _Kpp * _posError + _Kdp * _velError;   //期望的重心位置
    // _dWbd  = _kpw*rotMatToExp(_Rd*_G2B_RotMat) + _Kdw * (_wCmdGlobal - this->_lowState->getGyroGlobal());
    _dWbd  = _kpw*rotMatToExp<T>(_Rd*_G2B_RotMat) + _Kdw * (Vec3<T>(0,0,0) - this->_lowState->getGyroGlobal());
    // _dWbd(2) = 0;
    
    // _ddPcd(0) = saturation(_ddPcd(0), Vec2<T>(-30, 30));
    // _ddPcd(1) = saturation(_ddPcd(1), Vec2<T>(-1,1));
    // _ddPcd(2) = saturation(_ddPcd(2), Vec2<T>(-100, 100));

    // _dWbd(0) = saturation(_dWbd(0), Vec2<T>(-40, 40));
    // _dWbd(1) = saturation(_dWbd(1), Vec2<T>(-40, 40));
    // _dWbd(2) = saturation(_dWbd(2), Vec2<T>(-0.1, 0.1));
    _dWbd(2) = 0;

    _forceFeetGlobal = _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact);    //世界坐标下计算的足端力
    
    for(int i(0); i<4; ++i){   //摆动相   这个需要修改！！
        if((*_contact)(i) == 0){
            _forceFeetGlobal.col(i) = _KpSwing*(_B2G_RotMat*_posFeet2BGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing*(_B2G_RotMat*_velFeet2BGoal.col(i)-_velFeetGlobal.col(i));
            // _forceFeetGlobal.col(i) = Vec3<T>::Zero();
        }
    }
    // std::cout << "forceFeetGlobal: " << _forceFeetGlobal << std::endl;
    for(int i(0); i<4; ++i)
    {
        _forceFeetGlobal.col(i)(2) = saturation(_forceFeetGlobal.col(i)(2), Vec2<T>(-400, 400));
    }
    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;
    // std::cout << "forceFeetBody: " << _forceFeetBody << std::endl;
 
    _q = vec34ToVec12(this->_lowState->getQ());
    _qd = vec34ToVec12(this->_lowState->getQd());
    for(int i = 0; i<12; ++i)
    {
        if(std::isnan(_q(i))|| std::isnan(_qd(i)))
    {
        ROS_ERROR("_q and _qd nan in State_Trotting");
    }
    }
    
    _tau = _robModel->getTau(_q, _forceFeetBody);   //计算关节力矩,将车轮扭矩的力矩设置为0
    // std::cout << "tau_: " << _tau << std::endl;
    _tau_forward = vec34ToVec12(_robModel->calcJointTorqueForceTotal(vec12ToVec34(_q),vec12ToVec34(_qd),_qddGoal,vec12ToVec34(_tau)));
    // std::cout << "tau_forward: " << _tau_forward << std::endl;
}
template <typename T>
void State_Trotting<T>::calcQQd(){

    Vec34<T> _posFeet2B;
    _posFeet2B = _robModel->getFeet2BPositions(*this->_lowState,FrameType::BODY); //获取足端在机身坐标系下的位置,实时计算的,这个是非常准的！！
  
    // for(int i(0); i<4; ++i){
    //     _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _posBody);  
    //     _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody);     
    //     // std::cout<<"posFeet2BGoal: "<< _posFeetGlobalGoal.col(i) << std::endl;
    // }
    
    // if( _posError(0) < 0.1 && (_velError(0) < 0.05 || _vCmdBody(0) == 0.00))

    if( _vCmdBody(0) == 0.00)
    {
        _posFeet2BGoal = _robModel->getFeetPosIdeal();
        _velFeet2BGoal.setZero();
    }
    // _velFeet2BGoal.setZero();
    //  = _posFeet2B;
    // std::cout << "posFeet2BGoal: " << _posFeet2BGoal << std::endl;
    _qGoal = vec12ToVec34(_robModel->getQ(_posFeet2BGoal, FrameType::BODY));   //计算目标关节位置
    for (int i(0); i<4; ++i)   //车轮的位置
    {    
        _qGoal.col(i)(2) = this->_lowState->getQ().col(i)(2);
    }
    // std::cout << "_velFeet2BGoal: " << _velFeet2BGoal << std::endl;

    _qdGoal = vec12ToVec34(_robModel->getQd(_posFeet2B, _velFeet2BGoal,*this->_lowState, FrameType::BODY)); //获取期望关节速度
    //车轮的qdGoal应该是0
     for (int i(0); i<4; ++i)   //车轮的位置
    {    
        _qdGoal.col(i)(2) = 0;
    }
}
template <typename T>
bool State_Trotting<T>::checkStepOrNot(){
    if( (fabs(_vCmdBody(0)) > 0.00) ||
        (fabs(_vCmdBody(1)) > 1000) ||
        (fabs(_posError(0)) > 10.2) ||
        (fabs(_posError(1)) > 1000.5) ||
        (fabs(_velError(0)) > 0.5) ||
        (fabs(_velError(1)) > 1000) ||
        (fabs(_dYawCmd) > 10.5) ){
        return true;
        // return false;
    }else{
        return false;
    }
}
template <typename T>
void State_Trotting<T>::setHighCmd(T vx, T vy, T wz){
    _vCmdBody(0) = vx;
    _vCmdBody(1) = vy;
    _vCmdBody(1) = 0;
    _vCmdBody(2) = 0; 
    _dYawCmd = wz;
}
template <typename T>
FSMStateName State_Trotting<T>::checkChange(){
    if(this->_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(this->_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::TROTTING;
    }
}
template <typename T>
void State_Trotting<T>::PD_Callback(const std_msgs::Float32MultiArray& msg){
        _Kp = msg.data[0];
        _Kd = msg.data[1];
        _Kp_ = msg.data[2];
        _Kd_ = msg.data[3];
    
        _Kpp(0,0) =  msg.data[0];
        _Kdp(0,0) =  msg.data[1];
        
    }
template <typename T>
void State_Trotting<T>::initRecv(){    //订阅速度指令
        _PD_Sub = _nm.subscribe("/xingtian/PD_params", 1, &State_Trotting::PD_Callback, this);
    }
// template class State_Trotting<double>;
template class State_Trotting<float>;