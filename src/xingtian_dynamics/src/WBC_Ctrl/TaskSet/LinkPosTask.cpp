#include "WBC_Ctrl/TaskSet/LinkPosTask.hpp"
// (X, Y, Z)
// #include <Configuration.h>
#include <common/xingtianrobot.h>
// #include <Dynamics/Quadruped.h>
#include <Utilities/Utilities_print.h>

template <typename T> //确定连杆位置的类，摆动腿的足端位置设置
LinkPosTask<T>::LinkPosTask(const QuadrupedRobot<T>* robot, int link_idx,
                            bool virtual_depend)               //是否依赖虚拟关节
    : Task<T>(3),
      robot_sys_(robot),
      link_idx_(link_idx), 
      virtual_depend_(virtual_depend) {   
  TK::Jt_ = DMat<T>::Zero(TK::dim_task_, xingtian::dim_config);     //3*12 雅克比矩阵
  TK::JtDotQdot_ = DVec<T>::Zero(TK::dim_task_);                   //3*1

  _Kp = DVec<T>::Constant(TK::dim_task_, 150.);                   //3*1
  _Kd = DVec<T>::Constant(TK::dim_task_, 20.);                     //3*1
  _Kp_kin = DVec<T>::Constant(TK::dim_task_, 10.);                 //3*1
}

template <typename T>
LinkPosTask<T>::~LinkPosTask() {}

template <typename T>
bool LinkPosTask<T>::_UpdateCommand(const void* pos_des, const DVec<T>& vel_des,
                                    const DVec<T>& acc_des) {
  Vec3<T>* pos_cmd = (Vec3<T>*)pos_des;                   //获取输入的足端位置
  Vec3<T> link_pos;      

  link_pos = robot_sys_->_pGC[link_idx_];        //获取当前足端位置

  // X, Y, Z
  for (int i(0); i < 3; ++i) {
    TK::pos_err_[i] = _Kp_kin[i]* ( (*pos_cmd)[i] - link_pos[i] );    //获取位置误差
    TK::vel_des_[i] = vel_des[i];     
    TK::acc_des_[i] = acc_des[i];
  }

  // Op acceleration command
  for (size_t i(0); i < TK::dim_task_; ++i) {
    TK::op_cmd_[i] =
        _Kp[i] * TK::pos_err_[i] +
        _Kd[i] * (TK::vel_des_[i] - robot_sys_->_vGC[link_idx_][i]) +
        TK::acc_des_[i];
  }

  // printf("[Link Pos Task]\n");
  // pretty_print(acc_des, std::cout, "acc_des");
  // pretty_print(TK::pos_err_, std::cout, "pos_err_");
  // pretty_print(*pos_cmd, std::cout, "pos cmd");
  // pretty_print(robot_sys_->_vGC[link_idx_], std::cout, "velocity");
  // pretty_print(TK::op_cmd_, std::cout, "op cmd");
  // TK::op_cmd_.setZero();
  // pretty_print(TK::Jt_, std::cout, "Jt");

  return true;
}

template <typename T>
bool LinkPosTask<T>::_UpdateTaskJacobian() {                        //更新雅克比矩阵
  TK::Jt_ = robot_sys_->_Jc[link_idx_];                              //获取当前足端雅克比矩阵
  if (!virtual_depend_) {          //不会运行这里
    TK::Jt_.block(0, 0, 3, 6) = DMat<T>::Zero(3, 6);
  }
  return true;
}

template <typename T>
bool LinkPosTask<T>::_UpdateTaskJDotQdot() {                        //更新雅克比矩阵的微分
  TK::JtDotQdot_ = robot_sys_->_Jcdqd[link_idx_];
  return true;
}

template class LinkPosTask<double>;
template class LinkPosTask<float>;
