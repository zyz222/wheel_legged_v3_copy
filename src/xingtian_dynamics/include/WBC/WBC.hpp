//这个文件无需修改！！

#ifndef E490C057_868B_4932_8BDE_7FC5D4492084
#define E490C057_868B_4932_8BDE_7FC5D4492084
// 开始修改下一个！

#ifndef WHOLE_BODY_CONTROLLER
#define WHOLE_BODY_CONTROLLER

#include <Utilities/Utilities_print.h>
#include <Utilities/pseudoInverse.h>
#include "common/cppTypes.h"
#include <vector>
#include "WBC/ContactSpec.hpp"
#include "WBC/Task.hpp"
#include "control/CtrlComponents.h"
// Assume first 6 (or 3 in 2D case) joints are for the representation of
// a floating base.

#define WB WBC<T>
//被WBIC继承实现虚函数功能
template <typename T>
class WBC {
 public:
  WBC(size_t num_qdot) : num_act_joint_(num_qdot - 6), num_qdot_(num_qdot) {      //初始化自由度数量，前6个为机身自由度。
    Sa_ = DMat<T>::Zero(num_act_joint_, num_qdot_);    //12*18
    Sv_ = DMat<T>::Zero(6, num_qdot_);                  //6*18

    Sa_.block(0, 6, num_act_joint_, num_act_joint_).setIdentity();    // 
    Sv_.block(0, 0, 6, 6).setIdentity(); 
  }
  virtual ~WBC() {}

  virtual void UpdateSetting(const DMat<T>& A, const DMat<T>& Ainv,                        //定义虚函数，初始化惯性矩阵，惯性矩阵的逆，科氏力矩阵，和重力矩阵
                             const DVec<T>& cori, const DVec<T>& grav,
                             void* extra_setting = NULL) = 0;

  virtual void MakeTorque(DVec<T>& cmd, void* extra_input = NULL) = 0;                   // 计算力矩

 protected:
  // full rank fat matrix only   计算带权重的矩阵的伪逆矩阵
  void _WeightedInverse(const DMat<T>& J, const DMat<T>& Winv, DMat<T>& Jinv,
                        double threshold = 0.0001) {
    DMat<T> lambda(J * Winv * J.transpose());
    DMat<T> lambda_inv;
    pseudoInverse(lambda, threshold, lambda_inv);   //计算伪逆矩阵
    Jinv = Winv * J.transpose() * lambda_inv;
  }

  size_t num_act_joint_;               //执行器的数量，12
  size_t num_qdot_;                       // 机体系自由度数量，18

  DMat<T> Sa_;  // Actuated joint
  DMat<T> Sv_;  // Virtual joint

  DMat<T> A_;
  DMat<T> Ainv_;
  DVec<T> cori_;
  DVec<T> grav_;

  bool b_updatesetting_;
  bool b_internal_constraint_;
};

#endif


#endif /* E490C057_868B_4932_8BDE_7FC5D4492084 */
