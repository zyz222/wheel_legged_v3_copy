#include "WBC_Ctrl/TaskSet/BodyPosTask.hpp"
// (X, Y, Z)
// #include <Configuration.h>
#include <common/xingtianrobot.h>
// #include <Dynamics/Quadruped.h>
#include <Utilities/Utilities_print.h>

template <typename T>
BodyPosTask<T>::BodyPosTask(const QuadrupedRobot<T>* robot)               //机身位置的构造函数！！position
    : Task<T>(3), _robot_sys(robot) {  
  TK::Jt_ = DMat<T>::Zero(TK::dim_task_, xingtian::dim_config);             //雅可比矩阵
  TK::Jt_.block(0, 3, 3, 3).setIdentity();                                  //雅克比矩阵的初始化为单位矩阵
  TK::JtDotQdot_ = DVec<T>::Zero(TK::dim_task_);                            //雅克比矩阵0

  _Kp_kin = DVec<T>::Constant(TK::dim_task_, 1.);      //都是三维度
  _Kp = DVec<T>::Constant(TK::dim_task_, 150.);
  _Kd = DVec<T>::Constant(TK::dim_task_, 20.0);
}

template <typename T>
BodyPosTask<T>::~BodyPosTask() {}

template <typename T>
bool BodyPosTask<T>::_UpdateCommand(const void* pos_des, const DVec<T>& vel_des,      //更新指令，期望的位置，期望的线速度，期望的加速度
                                    const DVec<T>& acc_des) {
  Vec3<T>* pos_cmd = (Vec3<T>*)pos_des;                                 //期望的机身位置
  Vec3<T> link_pos = _robot_sys->_state.bodyPosition;                        //当前机身位置

  Quat<T> quat = _robot_sys->_state.bodyOrientation;                         //四元数
  Mat3<T> Rot = ori::quaternionToRotationMatrix(quat);                        //旋转矩阵

  SVec<T> curr_vel = _robot_sys->_state.bodyVelocity;                          //当前机身速度，
  curr_vel.tail(3) = Rot.transpose() * curr_vel.tail(3);                     //当前机身线速度转到局部坐标系下

  // X, Y, Z
  for (int i(0); i < 3; ++i) {
    TK::pos_err_[i] = _Kp_kin[i] * ((*pos_cmd)[i] - link_pos[i]);              //只是用于获取误差
    TK::vel_des_[i] = vel_des[i];
    TK::acc_des_[i] = acc_des[i];

    TK::op_cmd_[i] = _Kp[i] * ((*pos_cmd)[i] - link_pos[i]) +
                     _Kd[i] * (TK::vel_des_[i] - curr_vel[i + 3]) +            //机身的线速度差值
                     TK::acc_des_[i];
  }
  // Quat<T> quat = _robot_sys->_state.bodyOrientation;
  // Mat3<T> Rot = ori::quaternionToRotationMatrix(quat);
  // TK::pos_err_ = Rot * TK::pos_err_;
  // TK::vel_des_ = Rot * TK::vel_des_;
  // TK::acc_des_ = Rot * TK::acc_des_;

  // printf("[Body Pos Task]\n");
  // pretty_print(acc_des, std::cout, "acc_des");
  // pretty_print(TK::pos_err_, std::cout, "pos_err_");
  // pretty_print(curr_vel, std::cout, "curr_vel");
  // pretty_print(*pos_cmd, std::cout, "pos cmd");
  // pretty_print(TK::op_cmd_, std::cout, "op cmd");
  // pretty_print(TK::vel_des_, std::cout, "vel des");
  // pretty_print(TK::Jt_, std::cout, "Jt");

  return true;
}

template <typename T>
bool BodyPosTask<T>::_UpdateTaskJacobian() {                     //机身位置雅克比
  Quat<T> quat = _robot_sys->_state.bodyOrientation;             //四元数
  Mat3<T> Rot = ori::quaternionToRotationMatrix(quat);             //旋转矩阵
  TK::Jt_.block(0, 3, 3, 3) = Rot.transpose();                     //根姿态的雅克比是一样的
  // TK::Jt_.block(0,3, 3,3) = Rot;
  // pretty_print(TK::Jt_, std::cout, "Jt");
  // TK::Jt_.block(0,3, 3,3) = Rot*TK::Jt_.block(0,3,3,3);
  return true;
}

template <typename T>
bool BodyPosTask<T>::_UpdateTaskJDotQdot() {             //不考虑动态效应
  return true;
}

template class BodyPosTask<double>;
template class BodyPosTask<float>;
