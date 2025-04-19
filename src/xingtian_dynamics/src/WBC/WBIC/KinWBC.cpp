// 这个也不怎么需要修改！！！
#include "WBC/WBIC/KinWBC.hpp"
#include <Utilities/Utilities_print.h>
#include <Utilities/pseudoInverse.h>

template <typename T>                            //18 num qdot//运动学WBC，基于运动学计算机器人关节期望位置和速度命令，负责处理机器人任务列表和接触列表序列
KinWBC<T>::KinWBC(size_t num_qdot)
  :threshold_(0.001), num_qdot_(num_qdot), num_act_joint_(num_qdot - 6){     //threshold 用于伪逆运算中的阈值，处理奇异矩阵
  I_mtx = DMat<T>::Identity(num_qdot_, num_qdot_);     //I_mtx 单位矩阵 18*18,用于构建投影矩阵
}

template <typename T>
bool KinWBC<T>::FindConfiguration(                    //WBC运行的核心部分，根据任务列表和接触列表序列计算关节位置误差和速度误差
    const DVec<T>& curr_config, const std::vector<Task<T>*>& task_list,
    const std::vector<ContactSpec<T>*>& contact_list, DVec<T>& jpos_cmd,
    DVec<T>& jvel_cmd) {                                                 //curr_config 当前机器人配置，初始化是19维，应该是18维度，task_list 任务列表，contact_list 接触列表

  // Contact Jacobian Setup
  DMat<T> Nc(num_qdot_, num_qdot_); Nc.setIdentity();            //初始化投影矩阵Nc为单位矩阵，维度18*18
  if(contact_list.size() > 0){                       //如果接触列表不为空，则构建接触矩阵Jc
    DMat<T> Jc, Jc_i;                                //Jc 18*18，Jc_i 18*18
    contact_list[0]->getContactJacobian(Jc);          //将Jc赋值
    size_t num_rows = Jc.rows();                     //接触雅可比矩阵的行数

    for (size_t i(1); i < contact_list.size(); ++i) {
      contact_list[i]->getContactJacobian(Jc_i);               //获取第二个接触雅可比矩阵
      size_t num_new_rows = Jc_i.rows();
      Jc.conservativeResize(num_rows + num_new_rows, num_qdot_);  //调整大雅可比矩阵的大小，n*18
      Jc.block(num_rows, 0, num_new_rows, num_qdot_) = Jc_i;     //一直往下填充，从最左边开始填充
      num_rows += num_new_rows; 
    }
  
    // Projection Matrix
    _BuildProjectionMatrix(Jc, Nc);         //将任务空间投影到接触雅可比矩阵的零空间上，
  }

  // First Task
  DVec<T> delta_q, qdot;                           //位置误差、速度误差
  DMat<T> Jt, JtPre, JtPre_pinv, N_nx, N_pre;
  Task<T>* task = task_list[0];
  task->getTaskJacobian(Jt);                         //获取第一个任务的雅可比矩阵
  JtPre = Jt * Nc;                                   //将任务雅可比矩阵投影到接触雅可比的零空间上
  _PseudoInverse(JtPre, JtPre_pinv);                    //伪逆运算，计算雅可比矩阵的伪逆矩阵

  delta_q = -JtPre_pinv * (task->getPosError());         //计算位置误差  用于生成关节位置和速度命令

  qdot = -JtPre_pinv * (task->getDesVel());              //计算速度误差

  DVec<T> prev_delta_q = delta_q;                        //上一个任务的位置误差和速度误差
  DVec<T> prev_qdot = qdot;

  _BuildProjectionMatrix(JtPre, N_nx);                 //构建新的投影矩阵
  N_pre = Nc * N_nx;                                

  for (size_t i(1); i < task_list.size(); ++i) {             //循环处理任务列表
    task = task_list[i];

    task->getTaskJacobian(Jt);              
    JtPre = Jt * N_pre;

    _PseudoInverse(JtPre, JtPre_pinv);                           //伪逆运算，计算雅可比矩阵的伪逆矩阵
    delta_q =
        prev_delta_q + JtPre_pinv * (task->getPosError() - Jt * prev_delta_q);    //更新位置误差和速度误差
    qdot = prev_qdot + JtPre_pinv * (task->getDesVel() - Jt * prev_qdot);      //将当前任务的误差累加到之前的误差上

    // For the next task
    _BuildProjectionMatrix(JtPre, N_nx);                   //构建新的投影矩阵，用于下一个任务的雅可比矩阵投影
    N_pre *= N_nx;                                          //更新投影矩阵
    prev_delta_q = delta_q;
    prev_qdot = qdot;
  }
  for (size_t i(0); i < num_act_joint_; ++i) {                   //更新关节位置和速度命令，算出来的是误差
    jpos_cmd[i] = curr_config[i + 6] + delta_q[i + 6];
    jvel_cmd[i] = qdot[i + 6];

  }
  return true;
}

template <typename T>
void KinWBC<T>::_BuildProjectionMatrix(const DMat<T>& J, DMat<T>& N) {       //构建投影矩阵，将任何向量投影到雅克比矩阵的零空间上
  DMat<T> J_pinv;
  _PseudoInverse(J, J_pinv);
  N = I_mtx - J_pinv * J;                //18*18的单位矩阵-J-1*J，构建投影矩阵   I - J-1*J
}

template <typename T>
void KinWBC<T>::_PseudoInverse(const DMat<T> J, DMat<T>& Jinv) {       //伪逆运算，计算雅可比矩阵的伪逆矩阵
  pseudoInverse(J, threshold_, Jinv);               //传入矩阵J，阈值，和结果矩阵
}

template class KinWBC<double>;
template class KinWBC<float>;
