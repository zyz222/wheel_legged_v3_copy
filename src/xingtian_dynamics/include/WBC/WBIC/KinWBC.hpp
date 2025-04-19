// 这个文件应该也不怎么需要修改
#ifndef KINEMATICS_WHOLE_BODY_CONTROL
#define KINEMATICS_WHOLE_BODY_CONTROL

#include "WBC/ContactSpec.hpp"
#include "WBC/Task.hpp"
#include <vector>

template <typename T>
class KinWBC {
 public:
  KinWBC(size_t num_qdot);   //18
  ~KinWBC() {}

  bool FindConfiguration(const DVec<T>& curr_config,
                         const std::vector<Task<T>*>& task_list,
                         const std::vector<ContactSpec<T>*>& contact_list,
                         DVec<T>& jpos_cmd, DVec<T>& jvel_cmd);

  DMat<T> Ainv_;      //惯性矩阵的逆

 private:
  void _PseudoInverse(const DMat<T> J, DMat<T>& Jinv);    //计算雅可比矩阵的逆
  void _BuildProjectionMatrix(const DMat<T>& J, DMat<T>& N);    //构建雅可比投影矩阵

  double threshold_;          // 阈值 
  size_t num_qdot_;           //自由度数量18
  size_t num_act_joint_;       //执行器数量12
  DMat<T> I_mtx;         //单位矩阵
};
#endif
