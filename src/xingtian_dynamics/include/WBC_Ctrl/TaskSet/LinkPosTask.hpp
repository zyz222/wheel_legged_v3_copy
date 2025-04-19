#ifndef LINK_POS_TASK
#define LINK_POS_TASK

// (X, Y, Z)
#include <WBC/Task.hpp>

template <typename T>
class QuadrupedRobot;

template <typename T>
class LinkPosTask : public Task<T> {
 public:
  LinkPosTask(const QuadrupedRobot<T>*, int link_idx,    //设置摆动腿的足端位置
              bool virtual_depend = true);
  virtual ~LinkPosTask();

  DVec<T> _Kp, _Kd, _Kp_kin;   //位置控制比例增益，微分增益，位置误差相关项

 protected:
  // Update op_cmd_
  virtual bool _UpdateCommand(const void* pos_des, const DVec<T>& vel_des,
                              const DVec<T>& acc_des);                //更新命令
  // Update Jt_
  virtual bool _UpdateTaskJacobian();              //更新任务雅可比
  // Update JtDotQdot_
  virtual bool _UpdateTaskJDotQdot();   
  virtual bool _AdditionalUpdate() { return true; }     

  const QuadrupedRobot<T>* robot_sys_;
  int link_idx_;
  bool virtual_depend_;
};

#endif
