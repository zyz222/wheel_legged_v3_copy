#ifndef BODY_ORIENTATION_TASK
#define BODY_ORIENTATION_TASK
// (Rx, Ry, Rz)
#include <WBC/Task.hpp>

template <typename T>
class QuadrupedRobot;

template <typename T> 
class BodyOriTask : public Task<T> {     //机身姿态控制
 public:
  BodyOriTask(const QuadrupedRobot<T>*);
  virtual ~BodyOriTask();

  DVec<T> _Kp_kin;
  DVec<T> _Kp, _Kd;

 protected:
  // Update op_cmd_
  virtual bool _UpdateCommand(const void* pos_des, const DVec<T>& vel_des,
                              const DVec<T>& acc_des);
  // Update Jt_
  virtual bool _UpdateTaskJacobian();
  // Update JtDotQdot_
  virtual bool _UpdateTaskJDotQdot();
  virtual bool _AdditionalUpdate() { return true; }

  int link_idx_;
  bool virtual_depend_;
  const QuadrupedRobot<T>* _robot_sys;
};

#endif
