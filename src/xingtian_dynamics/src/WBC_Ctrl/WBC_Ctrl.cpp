
// 这是修改的第三个代码文件
#include "WBC_Ctrl/WBC_Ctrl.hpp"
#include <Utilities/Utilities_print.h>
#include <Utilities/Timer.h>
#include "common/xingtianrobot.h"
#include "WBC_Ctrl/WBC_Ctrl.hpp"
using namespace xingtian;
// using namespace Eigen;
using namespace linkID;

template <typename T>
WBC_Ctrl<T>::WBC_Ctrl(QuadrupedRobot<T> & model):
  _full_config(xingtian::num_act_joint + 7),     //19
  // _state(xingtian::dim_config),                //18
  _tau_ff(xingtian::num_act_joint),          //12
  _des_jpos(xingtian::num_act_joint),   //12
  _des_jvel(xingtian::num_act_joint),
  _model(model),  // 直接初始化
  _kin_wbc(new KinWBC<T>(xingtian::dim_config)),
  _wbic(new WBIC<T>(xingtian::dim_config, &_contact_list, &_task_list)),
  _wbic_data(new WBIC_ExtraData<T>())
   {
    // 函数体内其他初始化
  _iter = 0;
  _full_config.setZero();       //19*1
  _wbic_data->_W_floating = DVec<T>::Constant(6, 0.1);       //设置一个6维长度的向量，调整机身的的六维方向权重

  _wbic_data->_W_rf = DVec<T>::Constant(12, 1.);                       //设置一个12维长度的向量，调整关节力矩权重

  _Kp_joint.resize(xingtian::num_leg_joint, 5.);                             //设置关节控制权重，控制力矩 3*1
  _Kd_joint.resize(xingtian::num_leg_joint, 1.5);                           //设置关节控制权重，控制力矩 3*1

  _state.q = DVec<T>::Zero(xingtian::num_act_joint);                     //关节角度
  _state.qd = DVec<T>::Zero(xingtian::num_act_joint);                      //关节角速度
}
template<typename T>
WBC_Ctrl<T>::~WBC_Ctrl()
{
    delete _kin_wbc;
    delete _wbic;
    delete _wbic_data;

    typename std::vector<Task<T> *>::iterator iter = _task_list.begin();
    while (iter < _task_list.end())
    {
        delete (*iter);
        ++iter;
    }
    _task_list.clear();

    typename std::vector<ContactSpec<T> *>::iterator iter2 = _contact_list.begin();
    while (iter2 < _contact_list.end())
    {
        delete (*iter2);
        ++iter2;
    }
    _contact_list.clear();
}

template <typename T>
void WBC_Ctrl<T>::_ComputeWBC() {    //配置结束后，开始计算WBC控制数据
  // TEST
  _kin_wbc->FindConfiguration(_full_config, _task_list, _contact_list,
                              _des_jpos, _des_jvel);              //运动学WBC，根据接触序列、任务序列，计算位置误差和速度误差
                              
  // WBIC
  _wbic->UpdateSetting(_A, _Ainv, _coriolis, _grav);
  _wbic->MakeTorque(_tau_ff, _wbic_data);                       //基于当前的设置和接触点、任务数据、生成前馈力矩，输出期望的关节加速度
}

template<typename T>    //输入期望的WBC数据，和_data控制数据  包括状态估计，模型之类的数据！data是控制状态机数据
void WBC_Ctrl<T>::run(void* input, CtrlComponents<T> & _ctrlComp){                    //在站立平衡控制中，最后一部就会跳到这里来运行WBC控制器！！
  ++_iter;

  // Update Model
  _UpdateModel(*_ctrlComp.estimator,_ctrlComp.lowState);        // 更新_state，状态估计的数据和腿部控制器的数据

  // Task & Contact Update      input是输入的期望数据！！
  _ContactTaskUpdate(input, _ctrlComp);            //计算接触任务

  // WBC Computation
  _ComputeWBC();                                                                // 计算WBC控制数据，包括_tau_ff，以及_wbic_data
  
  // Update Leg Command
  _UpdateLegCMD(_ctrlComp);


}



template<typename T>
void WBC_Ctrl<T>::_UpdateLegCMD(CtrlComponents<T> & _ctrlComp){            //data控制状态机数据
  LegControllerCommand<T> *cmd = _ctrlComp.lowCmd->commands;       //更新腿部控制器命令，腿部控制器命令指针
  //Vec4<T> contact = _ctrlComp._stateEstimator->getResult().contactEstimate;

  for (size_t leg(0); leg < xingtian::num_leg; ++leg) {                    //4个腿
    cmd[leg].zero();        //4条腿的指令清零
    for (size_t jidx(0); jidx < xingtian::num_leg_joint; ++jidx) {       //3个关节
      cmd[leg].tauFeedForward[jidx] = -_tau_ff[xingtian::num_leg_joint * leg + jidx];   //赋值前馈力矩
      cmd[leg].qDes[jidx] = _des_jpos[xingtian::num_leg_joint * leg + jidx];           //赋值期望关节位置
      cmd[leg].qdDes[jidx] = _des_jvel[xingtian::num_leg_joint * leg + jidx];          //赋值期望关节速度

        cmd[leg].kpJoint(jidx, jidx) = _Kp_joint[jidx];             //3*3
        cmd[leg].kdJoint(jidx, jidx) = _Kd_joint[jidx];

    }
  }


  // Knee joint non flip barrier
  // for(size_t leg(0); leg<4; ++leg){                       //膝关节防止翻转机制
  //   if(cmd[leg].qDes[2] < 0.3){                          //如果膝关节角度小于0.3，则置为0.3
  //     cmd[leg].qDes[2] = 0.3;
  //   }
  //   // if(_ctrlComp.lowCmd->datas[leg].q[2] < 0.3){               //如果实际关节角度小于0.3，则根据当前膝关节位置调整前馈力矩，增加对膝关节的稳定性
  //   //   T knee_pos = _ctrlComp.lowCmd->datas[leg].q[2]; 
  //   //   cmd[leg].tauFeedForward[2] = 1./(knee_pos * knee_pos + 0.02);
  //   // }
  // }
}

template<typename T>
void WBC_Ctrl<T>::_UpdateModel(Estimator<T> & state_est,       //更新WBC控制所需机器人的状态，更新的就是data里边的数据
    const LowlevelState<T> * leg_data){                              //更新腿部控制器数据，包括关节位置和速度等

  _state.bodyOrientation = state_est.getQuat();   //获取机身的四元数                     //更新机身姿态     
  _state.bodyPosition = state_est.cheater_getPosition();      //获取机身位置                           //更新机身位置
  _state.bodyVelocity.head(3) = state_est.getomega();  //获取机身角速度
  _state.bodyVelocity.tail(3) = state_est.cheater_getVelocity();//获取机身线速度
 
  // for(int i = 0;i<4;i++)
  // {
    _state.q = vec34ToVec12(leg_data->getQ());     //获取所有关节角度，FL RL RR FR
    _state.qd = vec34ToVec12(leg_data->getQd());   //获取所有关节角速度
  // } 
  for(int i = 0;i<12;i++)
  { _full_config[i + 6] = _state.q[i];}             

  _model.setState(_state); //将更新后的state传递给机器人模型 ，只有一个传递和标志，这里没有计算！

  _model.contactJacobians();                 //计算关节位置的雅克比矩阵
  _model.massMatrix(); 
                                              //计算质量矩阵，惯性矩阵
  _model.generalizedGravityForce();           //计算重力作用向量
  // std::cout<<"_model.generalizedCoriolisForce()" << _model.generalizedGravityForce()<<std::endl;
  _model.generalizedCoriolisForce();         //计算 Coriolis作用向量，科氏力和离心力，与关节速度相关
  // std::cout<<"_model.generalizedCoriolisForce()" << _model.generalizedCoriolisForce()<<std::endl;

  
  _A = _model.getMassMatrix();         
  // std::cout<<"A"<<_A<<std::endl;                    
  _grav = _model.getGravityForce();
  // std::cout<<"grav"<<_grav<<std::endl;
  _coriolis = _model.getCoriolisForce();
  // std::cout<<"coriolis"<<_coriolis<<std::endl;
  _Ainv = _A.inverse();                        // 计算质量矩阵的逆    逆动力学的方法！！
}


// template class WBC_Ctrl<float>;
template class WBC_Ctrl<double>;
