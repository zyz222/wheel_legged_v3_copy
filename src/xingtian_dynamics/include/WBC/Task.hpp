
#ifndef WBC_TASK
#define WBC_TASK

#include "common/cppTypes.h"
#include <cstddef>
#include "common/Quadruped.h"
#define TK Task<T>

template <typename T>
class Task {                   //WBC控制的任务类
 public:
  Task(size_t dim)
      : b_set_task_(false),         //表示任务还未被设置
        dim_task_(dim),      //dim 维度   任务维度
        op_cmd_(dim),         //op_cmd_ 操作命令向量
        pos_err_(dim),         //位置误差,在姿态控制任务中，就是姿态误差
        vel_des_(dim),          //速度误差
        acc_des_(dim) {}         //加速度误差

  virtual ~Task() {}

  void getCommand(DVec<T>& op_cmd) { op_cmd = op_cmd_; }  //获取操作命令
  void getTaskJacobian(DMat<T>& Jt) { Jt = Jt_; }           //获取雅可比矩阵
  void getTaskJacobianDotQdot(DVec<T>& JtDotQdot) { JtDotQdot = JtDotQdot_; }    //获取雅可比矩阵的时间导数乘速度向量，表示的是科氏力部分。用于准确计算所需的控制力。

  bool UpdateTask(const void* pos_des, const DVec<T>& vel_des,           //更新任务，用内部的虚函数依次更新任务的雅可比矩阵、雅可比矩阵的导数乘以速度向量、
                  const DVec<T>& acc_des) {                          //操作命令，并执行额外的更新操作
    _UpdateTaskJacobian();
    _UpdateTaskJDotQdot();
    _UpdateCommand(pos_des, vel_des, acc_des);
    _AdditionalUpdate();
    b_set_task_ = true;
    return true;
  }

  bool IsTaskSet() { return b_set_task_; }                   //查询任务是否被设置
  size_t getDim() { return dim_task_; }                     //获取任务维度
  void UnsetTask() { b_set_task_ = false; }           

  const DVec<T>& getPosError() { return pos_err_; }                 //获取位置误差
  const DVec<T>& getDesVel() { return vel_des_; }                   //获取期望速度
  const DVec<T>& getDesAcc() { return acc_des_; }                   //获取期望加速度

 protected:            //受保护的虚函数，必须在子类中实现
  // Update op_cmd_
  virtual bool _UpdateCommand(const void* pos_des, const DVec<T>& vel_des,
                              const DVec<T>& acc_des) = 0;
  // Update Jt_
  virtual bool _UpdateTaskJacobian() = 0;
  // Update JtDotQdot_
  virtual bool _UpdateTaskJDotQdot() = 0;
  // Additional Update (defined in child classes)
  virtual bool _AdditionalUpdate() = 0;

  bool b_set_task_;                             //标记任务是否设置
  size_t dim_task_;                              //任务维度（即任务所涉及的自由度数量）

  DVec<T> op_cmd_;                               //操作命令向量，用于存储任务生成的控制命令
  DVec<T> JtDotQdot_;                           //任务雅克比矩阵的乘积，雅可比矩阵的时间导数乘以关节速度向量，用于动力学计算。
  DMat<T> Jt_;                                  //任务雅克比矩阵，用于计算任务空间控制命令。

  DVec<T> pos_err_;                            //位置误差向量，
  DVec<T> vel_des_;                              //速度目标向量，
  DVec<T> acc_des_;                               //加速度目标向量，
};

#endif

