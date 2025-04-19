// 这个也不怎么需要修改！！
#ifndef LOCOMOTION_CONTROLLER
#define LOCOMOTION_CONTROLLER

#include "WBC_Ctrl/WBC_Ctrl.hpp"
// 都是期望的数据！！
template<typename T>                        //wbc控制器的数据类型
class LocomotionCtrlData{                        //在站立平衡控制器中初始化了这个类！！！！
  public:
    Vec3<T> pBody_des;                          //位置
    Vec3<T> vBody_des;                          //速度
    Vec3<T> aBody_des;                          //加速度
    Vec3<T> pBody_RPY_des;                         //期望的姿态  RPY
    Vec3<T> vBody_Ori_des;                         //期望姿态角速度

    Vec3<T> pFoot_des[4];                          //足端位置
    Vec3<T> vFoot_des[4];                           //足端速度
    Vec3<T> aFoot_des[4];                           //足端加速度
    Vec3<T> Fr_des[4];                                 //期望足端力

    Vec4<T> contact_state;                             //足端接触状态
};

template<typename T>
class LocomotionCtrl: public WBC_Ctrl<T>{                          //在站立平衡控制器中初始化了这个类！！！！
  public:
    LocomotionCtrl(QuadrupedRobot<T> model);
    virtual ~LocomotionCtrl();

  protected:
    virtual void _ContactTaskUpdate(void * input, CtrlComponents<T> & data);
    // virtual void _ContactTaskUpdateTEST(void * input, CtrlComponents<T> & data);
    // void _ParameterSetup(const MIT_UserParameters* param);
    void _ParameterSetup(const LowlevelCmd<T>* param);
    void _CleanUp();


    LocomotionCtrlData<T>* _input_data;

    Task<T>* _body_pos_task;
    Task<T>* _body_ori_task;

    Task<T>* _foot_task[4];                 //摆动相的足端任务
    ContactSpec<T>* _foot_contact[4];    //足端接触判断

    Vec3<T> pre_foot_vel[4];

    Vec3<T> _Fr_result[4];
    Quat<T> _quat_des;
};

#endif

