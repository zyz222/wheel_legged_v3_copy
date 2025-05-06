#include "WBC_Ctrl/TaskSet/BodyOriTask.hpp"
// (Rx, Ry, Rz)

// #include <Configuration.h>
#include <common/xingtianrobot.h>
// #include <Dynamics/Quadruped.h>
#include "common/orientation_tools.h"
#include <Utilities/Utilities_print.h>

// 姿态控制任务
template <typename T>
BodyOriTask<T>::BodyOriTask(const QuadrupedRobot<T>* robot)           //构造函数，传入机器人动力学模型，设置任务维度为3，
    : Task<T>(3), _robot_sys(robot) {
  TK::Jt_ = DMat<T>::Zero(TK::dim_task_, xingtian::dim_config);    // Jt_ = 3x18     初始化雅克比矩阵为0。    //任务雅可比矩阵
  TK::Jt_.block(0, 0, 3, 3).setIdentity();                                 //左上角矩阵设置为单位矩阵。
  TK::JtDotQdot_ = DVec<T>::Zero(TK::dim_task_);                           //初始化为零向量，长度为3

  _Kp_kin = DVec<T>::Constant(TK::dim_task_, 50.);                     //设置姿态误差比例为1
  _Kp = DVec<T>::Constant(TK::dim_task_, 150.);                       //设置姿态控制比例为50
  _Kd = DVec<T>::Constant(TK::dim_task_, 10.);                              //设置姿态控制微分增益1
}

template <typename T>
BodyOriTask<T>::~BodyOriTask() {}

template <typename T>
bool BodyOriTask<T>::_UpdateCommand(const void* pos_des, const DVec<T>& vel_des,
                                    const DVec<T>& acc_des) {               //更新命令，传入目标姿态，目标速度，目标加速度
  Quat<T>* ori_cmd = (Quat<T>*)pos_des;                                  //期望的姿态，四元数类型，4*1
  Quat<T> link_ori = (_robot_sys->_state.bodyOrientation);                 //获取当前机器人的姿态，四元数类型，4*1      

  Quat<T> link_ori_inv;                                     //定义一个四元数，用于存储当前机器人的姿态的逆，四元数类型，4*1
  link_ori_inv[0] = link_ori[0];
  link_ori_inv[1] = -link_ori[1];
  link_ori_inv[2] = -link_ori[2];
  link_ori_inv[3] = -link_ori[3];                       //四元数的共扼（逆）
  // link_ori_inv /= link_ori.norm();

  // Explicit because operational space is in global frame
  Quat<T> ori_err = ori::quatProduct(*ori_cmd, link_ori_inv);             //计算姿态误差，表示从一个姿态到另一个姿态的旋转，表示的是相对旋转
  if (ori_err[0] < 0.) {                                                  //如果四元数的共轭小于0，则取反
    ori_err *= (-1.);                                                      //取反
  }                                                        
  Vec3<T> ori_err_so3;                                                   //定义一个3*1的向量，用于存储姿态误差的so3形式
  ori::quaternionToso3(ori_err, ori_err_so3);                    //将四元数误差转为旋量so3，得到姿态误差的向量表示
  SVec<T> curr_vel = _robot_sys->_state.bodyVelocity;                       //获取当前机器人的机身速度，6*1的向量，角速度、加速度
  
  // Configuration space: Local
  // Operational Space: Global
  Mat3<T> Rot = ori::quaternionToRotationMatrix(link_ori);                     ///将当前机器人的姿态转换为旋转矩阵，得到姿态的旋转表示   
  Vec3<T> vel_err = Rot.transpose()*(TK::vel_des_ - curr_vel.head(3));       //定义将期望角速度和当前角速度差异转换到局部坐标系下，全局转到局部

  // Rx, Ry, Rz                                            //这里应该是谢错了！！！！
  for (int i(0); i < 3; ++i) {                              
    TK::pos_err_[i] = _Kp_kin[i] * ori_err_so3[i];               //将姿态误差*比例增益，运动学的姿态误差
    TK::vel_des_[i] = vel_des[i];                                  //将期望速度赋值给当前速度
    TK::acc_des_[i] = acc_des[i];                                  //将期望加速度赋值给当前加速度

    TK::op_cmd_[i] = _Kp[i] * ori_err_so3[i] +                         //将姿态误差*比例增益+姿态微分误差*微分增益+姿态加速度赋值给当前命令
                     _Kd[i] * vel_err[i] + TK::acc_des_[i];
  }
   //printf("[Body Ori Task]\n");
   //pretty_print(TK::pos_err_, std::cout, "pos_err_");
   //pretty_print(*ori_cmd, std::cout, "des_ori");
   //pretty_print(link_ori, std::cout, "curr_ori");
   //pretty_print(ori_err, std::cout, "quat_err");

  // pretty_print(link_ori_inv, std::cout, "ori_inv");
  // pretty_print(ori_err, std::cout, "ori_err");
  // pretty_print(*ori_cmd, std::cout, "cmd");
  // pretty_print(acc_des, std::cout, "acc_des");
  // pretty_print(TK::Jt_, std::cout, "Jt");

  return true;
}

template <typename T>
bool BodyOriTask<T>::_UpdateTaskJacobian() {                         //更新任务雅可比矩阵，先更新的这个
  Quat<T> quat = _robot_sys->_state.bodyOrientation;             //获取当前机器人的姿态，四元数类型，4*1
  Mat3<T> Rot = ori::quaternionToRotationMatrix(quat);
  TK::Jt_.block(0, 0, 3, 3) = Rot.transpose();
  //pretty_print(Rot, std::cout, "Rot mat");
  return true;
}

template <typename T>
bool BodyOriTask<T>::_UpdateTaskJDotQdot() {                 //为0，不考虑动态效应
  return true;
}

template class BodyOriTask<double>;
template class BodyOriTask<float>;
