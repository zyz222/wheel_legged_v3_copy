#ifndef BEAC9B3F_1318_4EE0_938E_D28466AB9DCE
#define BEAC9B3F_1318_4EE0_938E_D28466AB9DCE
#ifndef WHOLE_BODY_IMPULSE_CONTROL_H
#define WHOLE_BODY_IMPULSE_CONTROL_H

#include <Utilities/Utilities_print.h>
#include "thirdParty/quadProgpp/QuadProg++.hh"
#include <WBC/ContactSpec.hpp>
#include <WBC/Task.hpp>
#include <WBC/WBC.hpp>

template <typename T>
class WBIC_ExtraData {                   // Extra data for WBIC 
 public:                             //存储WBC的外部数据，如权重和优化结果
  // Output
  DVec<T> _opt_result;               //优化结果
  DVec<T> _qddot;                    //加速度  
  DVec<T> _Fr;                       //反作用力向量  

  // Input
  DVec<T> _W_floating;               //权重向量
  DVec<T> _W_rf;                     //反作用力权重

  WBIC_ExtraData() {}
  ~WBIC_ExtraData() {}
};
//继承WBC类
template <typename T>
class WBIC : public WBC<T> {                   //用于WBC控制生成力矩命令的类，结合运动学和动力学信息
 public:
  WBIC(size_t num_qdot, const std::vector<ContactSpec<T>*>* contact_list,      //自由度18，接触序列，任务序列
       const std::vector<Task<T>*>* task_list);                       //接触序列，任务序列
  virtual ~WBIC() {}
 //更新WBC设置
  virtual void UpdateSetting(const DMat<T>& A, const DMat<T>& Ainv,
                             const DVec<T>& cori, const DVec<T>& grav,  
                             void* extra_setting = NULL);           

  virtual void MakeTorque(DVec<T>& cmd, void* extra_input = NULL);

 private:
  const std::vector<ContactSpec<T>*>* _contact_list;
  const std::vector<Task<T>*>* _task_list;

  void _SetEqualityConstraint(const DVec<T>& qddot);
  void _SetInEqualityConstraint();
  void _ContactBuilding();

  void _GetSolution(const DVec<T>& qddot, DVec<T>& cmd);
  void _SetCost();
  void _SetOptimizationSize();

  size_t _dim_opt;      // Contact pt delta, First task delta, reaction force   优化变量的维度
  size_t _dim_eq_cstr;  // equality constraints                                等是约束的维度

  size_t _dim_rf;  // inequality constraints                                     反作用力的维度
  size_t _dim_Uf;                                                                // inequality constraints   不等式约束的维度

  size_t _dim_floating;                                                           // 浮动基座维度 6

  WBIC_ExtraData<T>* _data;                                                       //WBC的外部存储数据
  
  quadprogpp::Vector<double> z;           //二次规划中的解向量
  // Cost
  quadprogpp::Matrix<double> G;          //二次规划中的成本矩阵
  quadprogpp::Vector<double> g0;             //二次规划中的成本向量
  
  // Equality
  quadprogpp::Matrix<double> CE;           //二次规划中的等式约束矩阵
  quadprogpp::Vector<double> ce0;           //二次规划中的等式约束向量

  // Inequality
  quadprogpp::Matrix<double> CI;            //二次规划中的不等式约束矩阵
  quadprogpp::Vector<double> ci0;           //二次规划中的不等式约束向量

  DMat<T> _dyn_CE;                          //动态等式约束矩阵和向量
  DVec<T> _dyn_ce0;
  DMat<T> _dyn_CI;                          //动态不等式约束矩阵和向量
  DVec<T> _dyn_ci0;

  DMat<T> _eye;                          //投影矩阵，单位矩阵18*18
  DMat<T> _eye_floating;                 //投影矩阵，单位矩阵6*6，用于浮动基座的六个自由度

  DMat<T> _S_delta;
  DMat<T> _Uf;                         //不等式约束矩阵
  DVec<T> _Uf_ieq_vec;                  //不等式约束向量

  DMat<T> _Jc;                       //接触雅可比矩阵
  DVec<T> _JcDotQdot;                  //接触雅可比矩阵乘以速度
  DVec<T> _Fr_des;                       //期望反作用力向量

  DMat<T> _B;
  DVec<T> _c;
  DVec<T> task_cmd_;
};

#endif




#endif /* BEAC9B3F_1318_4EE0_938E_D28466AB9DCE */
