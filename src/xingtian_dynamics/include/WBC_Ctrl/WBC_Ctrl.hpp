//这个也已经修改完了！！
#ifndef WBC_CONTROLLER_H
#define WBC_CONTROLLER_H

#include "control/CtrlComponents.h"      //状态机控制器数据类
#include "common/xingtianrobot.h"
#include "WBC/WBC.hpp"
#include "common/cppTypes.h"
#include "WBC/WBIC/WBIC.hpp"
#include "WBC/WBIC/KinWBC.hpp"
#include "control/xingtian_parameters.h"
#define WBCtrl WBC_Ctrl<T>

// class KeyBoard<T>;        //获取用户输入
// class XingtianParameters<T>;
template<typename T>
class WBC_Ctrl{
  public:
    WBC_Ctrl(QuadrupedRobot<T> &model);
    virtual ~WBC_Ctrl();

    void run(void * input, CtrlComponents<T> & _ctrlComp);
    void setFloatingBaseWeight(const T & weight){                   //T是一个模板参数，表示可以接收任意类型的数据，设置浮动基座质量权重
      _wbic_data->_W_floating = DVec<T>::Constant(6, weight);              //whole body impedance controller,DVec<T>是一个动态向量，长度可以指定
    }

  protected:
    virtual void _ContactTaskUpdate(void * input, CtrlComponents<T> & data) = 0;
    // virtual void _ContactTaskUpdateTEST(void * input, CtrlComponents<T> & data){
    //   (void)input;
    //   (void)data;
    // }
    void _UpdateModel(Estimator<T> & state_est, const LowlevelState<T> * lowState);  //获取状态值和腿部控制器数据
    void _UpdateLegCMD(CtrlComponents<T> & data);
    void _ComputeWBC();

    KinWBC<T>* _kin_wbc;                               //运动学WBC，用于处理运动学相关计算
    WBIC<T>* _wbic;                                    //WBC控制器，用于计算控制力，生成关节控制力
    WBIC_ExtraData<T>* _wbic_data;                     //WBIC控制器的数据

    QuadrupedRobot<T> _model;                        //机器人动力学模型
    LowlevelState<T>* lowState;     //腿部关节数据
    std::vector<ContactSpec<T> * > _contact_list;   //接触列表
    std::vector<Task<T> * > _task_list;          //容器类型，任务列表

    CtrlComponents<T> *_ctrlComp;    //控制器组件，不知道用啥就用它

    DMat<T> _A;               //通常表示惯性矩阵
    DMat<T> _Ainv;            //通常表示惯性矩阵的逆
    DVec<T> _grav;            //通常表示重力向量
    DVec<T> _coriolis;          //通常表示科里olis向量

    FBModelState<T> _state;                         //机器人当前状态，包括机身位置、速度、关节角度、关节速度等

    DVec<T> _full_config;                               //机器人的全局配置向量，包括关节角度、关节速度等   19*1
    DVec<T> _tau_ff;                          //前馈力矩向量
    DVec<T> _des_jpos;                          //期望关节角度向量
    DVec<T> _des_jvel;                          //期望关节速度向量

    std::vector<T> _Kp_joint, _Kd_joint;                   //关节比例微分增益
    //std::vector<T> _Kp_joint_swing, _Kd_joint_swing;

    unsigned long long _iter;                              //迭代次数

    
};


#endif

