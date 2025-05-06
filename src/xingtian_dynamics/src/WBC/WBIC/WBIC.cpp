
#include "WBC/WBIC/WBIC.hpp"
#include <Utilities/Timer.h>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/SVD>

template <typename T>
WBIC<T>::WBIC(size_t num_qdot, const std::vector<ContactSpec<T>*>* contact_list,      //自由度18 接触 任务序列
    const std::vector<Task<T>*>* task_list)
  : WBC<T>(num_qdot), _dim_floating(6) {                 //18个自由度，初始化WBC类
    _contact_list = contact_list;
    _task_list = task_list;

    _eye = DMat<T>::Identity(WB::num_qdot_, WB::num_qdot_);
    _eye_floating = DMat<T>::Identity(_dim_floating, _dim_floating);
  }

template <typename T>
void WBIC<T>::MakeTorque(DVec<T>& cmd, void* extra_input) {     //核心功能，通过二次规划求解来计算机器人关节的力矩命令。输入指令和外部输入
  if (!WB::b_updatesetting_) {
    printf("[Wanning] WBIC setting is not done\n");
  }
  if (extra_input) _data = static_cast<WBIC_ExtraData<T>*>(extra_input);   //如果提供了外部输入，就将其转换并存储在data中。

  // resize G, g0, CE, ce0, CI, ci0
  _SetOptimizationSize();          //设置变量和优化的数量。
  _SetCost();                    //设置优化问题的成本函数矩阵和向量

  DVec<T> qddot_pre;              //存储关节加速度的初始估计


  DMat<T> JcBar;                  //加权逆雅可比矩阵
  DMat<T> Npre;                  //投影矩阵，表示任务空间的投影

  if (_dim_rf > 0) {       //处理接触点，如果有反作用力约束
    // Contact Setting 
    _ContactBuilding();              //构建接触点雅可比矩阵和反作用力约束


    // Set inequality constraints
    _SetInEqualityConstraint();     //设置不等式约束条件。
    WB::_WeightedInverse(_Jc, WB::Ainv_, JcBar);   //计算加权逆雅可比矩阵
    qddot_pre = JcBar * (-_JcDotQdot);                 //计算关节初始加速度
    Npre = _eye - JcBar * _Jc;                         //计算投影矩阵，后续任务的独立性处理
    // pretty_print(JcBar, std::cout, "JcBar");
    // pretty_print(_JcDotQdot, std::cout, "JcDotQdot");
    // pretty_print(qddot_pre, std::cout, "qddot 1");
  } else {
    qddot_pre = DVec<T>::Zero(WB::num_qdot_);          //初始化关节加速度为0
    Npre = _eye;
  }

  // Task
  Task<T>* task;
  DMat<T> Jt, JtBar, JtPre;
  DVec<T> JtDotQdot, xddot;
  //处理任务列表
  for (size_t i(0); i < (*_task_list).size(); ++i) {
    task = (*_task_list)[i];

    task->getTaskJacobian(Jt);                        //获取当前任务的雅可比
    task->getTaskJacobianDotQdot(JtDotQdot);           //获取当前任务的雅可比矩阵时间导数乘以关节速度向量
    task->getCommand(xddot);                            //获取当前任务的指令（期望的加速度）

    JtPre = Jt * Npre;                                 //计算当前任务的雅可比矩阵的投影矩阵
    WB::_WeightedInverse(JtPre, WB::Ainv_, JtBar);             //计算加权逆雅可比矩阵JtBar

    qddot_pre += JtBar * (xddot - JtDotQdot - Jt * qddot_pre);      //计算关节加速度
    Npre = Npre * (_eye - JtBar * JtPre);                         //更新投影矩阵

    // pretty_print(xddot, std::cout, "xddot");
    // pretty_print(JtDotQdot, std::cout, "JtDotQdot");
    // pretty_print(qddot_pre, std::cout, "qddot 2");
    // pretty_print(Jt, std::cout, "Jt");
    // pretty_print(JtPre, std::cout, "JtPre");
    // pretty_print(JtBar, std::cout, "JtBar");
  }

  // Set equality constraints
  _SetEqualityConstraint(qddot_pre);                      //设置等式约束条件。

  // printf("G:\n");
  // std::cout<<G<<std::endl;
  // printf("g0:\n");
  // std::cout<<g0<<std::endl;

  // Optimization
  // Timer timer;
  T f = solve_quadprog(G, g0, CE, ce0, CI, ci0, z);                     //求解二次规划问题
  // std::cout<<"\n wbic old time: "<<timer.getMs()<<std::endl;
  (void)f;

  // pretty_print(qddot_pre, std::cout, "qddot_cmd");
  for (size_t i(0); i < _dim_floating; ++i) qddot_pre[i] += z[i];     //将优化结果Z加到关节加速度上 6维
  _GetSolution(qddot_pre, cmd);                                    //将更新的关节加速度转换为力矩命令cmd

  _data->_opt_result = DVec<T>(_dim_opt);                               //将优化结果存储在WBIC_ExtraData中
  for (size_t i(0); i < _dim_opt; ++i) {
    _data->_opt_result[i] = z[i];
  }

}

template <typename T>
/*
    设置等式约束矩阵 _dyn_CE 的前 dim_floating 列为动力学矩阵 A_ 的对应块。
    设置等式约束矩阵 _dyn_CE 的后 dim_rf 列为 -Sv_ * Jc^T，其中 Sv_ 是一个权重矩阵，Jc^T 是接触雅可比矩阵的转置。
    设置等式约束向量 _dyn_ce0 为 -Sv_ * (A * qddot + coriolis + gravity - Jc^T * Fr_des)，确保机器人动力学平衡。

如果没有反作用力约束，则仅设置动力学矩阵 A_ 的对应块，并设置等式约束向量 _dyn_ce0 为 -Sv_ * (A * qddot + coriolis + gravity)
*/
void WBIC<T>::_SetEqualityConstraint(const DVec<T>& qddot) {                  //设 置等式约束条件。
  if (_dim_rf > 0) {                                        //检查反作用力约束
    _dyn_CE.block(0, 0, _dim_eq_cstr, _dim_floating) =        //等式约束维度*6
      WB::A_.block(0, 0, _dim_floating, _dim_floating);                //6*6
    _dyn_CE.block(0, _dim_floating, _dim_eq_cstr, _dim_rf) =          //反作用力约束
      -WB::Sv_ * _Jc.transpose();
    _dyn_ce0 = -WB::Sv_ * (WB::A_ * qddot + WB::cori_ + WB::grav_ -      //单腿动力学方程，等式约束向量
        _Jc.transpose() * _Fr_des);
  } else {
    _dyn_CE.block(0, 0, _dim_eq_cstr, _dim_floating) =                      //如果没有反作用力约束，则仅设置动力学矩阵 A_ 的对应块，并设置等式约束向量 _dyn_ce0 为 -Sv_ * (A * qddot + coriolis + gravity)
      WB::A_.block(0, 0, _dim_floating, _dim_floating);
    _dyn_ce0 = -WB::Sv_ * (WB::A_ * qddot + WB::cori_ + WB::grav_);         //单腿动力学方程，等式约束向量
  }

  for (size_t i(0); i < _dim_eq_cstr; ++i) {                 //设置等式约束矩阵和向量，。
    for (size_t j(0); j < _dim_opt; ++j) {         //遍历每个等式约束，将动态等式约束矩阵复制到CE中
      CE[j][i] = _dyn_CE(i, j);
    }
    ce0[i] = -_dyn_ce0[i];                               //等式约束向量也放进去
  }
  // pretty_print(_dyn_CE, std::cout, "WBIC: CE");
  // pretty_print(_dyn_ce0, std::cout, "WBIC: ce0");
}

template <typename T>
void WBIC<T>::_SetInEqualityConstraint() {                                  //设置不等式约束条件，摩擦锥约束
  _dyn_CI.block(0, _dim_floating, _dim_Uf, _dim_rf) = _Uf;                     //将不等式约束矩阵 赋值给动态不等式的相应块
  _dyn_ci0 = _Uf_ieq_vec - _Uf * _Fr_des;                              //设置动态不等式约束向量，确保反作用力满足不等式约束

  for (size_t i(0); i < _dim_Uf; ++i) {                   //构建不等式约束矩阵和向量
    for (size_t j(0); j < _dim_opt; ++j) {
      CI[j][i] = _dyn_CI(i, j);
    }
    ci0[i] = -_dyn_ci0[i];
  }
  // pretty_print(_dyn_CI, std::cout, "WBIC: CI");
  // pretty_print(_dyn_ci0, std::cout, "WBIC: ci0");
}

template <typename T>
void WBIC<T>::_ContactBuilding() {                     //用于构建和整合所有接触点的雅可比矩阵、接触力学约束和期望的反作用力。
  DMat<T> Uf;                                        //不等式约束矩阵
  DVec<T> Uf_ieq_vec;                                //不等式约束向量
  // Initial
  DMat<T> Jc;                                            //接触点雅可比矩阵
  DVec<T> JcDotQdot;                                      //接触点雅可比矩阵的时间导数乘以关节速度向量 。加速度
  size_t dim_accumul_rf, dim_accumul_uf;                  //累积的反作用力维度，累积不等式约束维度
  (*_contact_list)[0]->getContactJacobian(Jc);                       //获取第一个接触点雅可比矩阵和雅可比矩阵的时间导数乘以关节速度向量
  (*_contact_list)[0]->getJcDotQdot(JcDotQdot);
  (*_contact_list)[0]->getRFConstraintMtx(Uf);                          //获取第一个接触点的不等式约束矩阵和不等式约束向量
  (*_contact_list)[0]->getRFConstraintVec(Uf_ieq_vec);             //获取反作用力约束向量

  dim_accumul_rf = (*_contact_list)[0]->getDim();                          //获取第一个接触点的反作用力维度和不等式约束维度。累积的反作用力维度
  dim_accumul_uf = (*_contact_list)[0]->getDimRFConstraint();

  _Jc.block(0, 0, dim_accumul_rf, WB::num_qdot_) = Jc;                         //将获取的数据赋值到相关变量中         
  _JcDotQdot.head(dim_accumul_rf) = JcDotQdot;
  _Uf.block(0, 0, dim_accumul_uf, dim_accumul_rf) = Uf;
  _Uf_ieq_vec.head(dim_accumul_uf) = Uf_ieq_vec;    
  _Fr_des.head(dim_accumul_rf) = (*_contact_list)[0]->getRFDesired();

  size_t dim_new_rf, dim_new_uf;                                    //获取新接触点的反作用力维度和不等式约束维度。

  for (size_t i(1); i < (*_contact_list).size(); ++i) {                          
    (*_contact_list)[i]->getContactJacobian(Jc);                           
    (*_contact_list)[i]->getJcDotQdot(JcDotQdot);                                          

    dim_new_rf = (*_contact_list)[i]->getDim();
    dim_new_uf = (*_contact_list)[i]->getDimRFConstraint();

    // Jc append
    _Jc.block(dim_accumul_rf, 0, dim_new_rf, WB::num_qdot_) = Jc;

    // JcDotQdot append
    _JcDotQdot.segment(dim_accumul_rf, dim_new_rf) = JcDotQdot;

    // Uf
    (*_contact_list)[i]->getRFConstraintMtx(Uf);
    _Uf.block(dim_accumul_uf, dim_accumul_rf, dim_new_uf, dim_new_rf) = Uf;

    // Uf inequality vector
    (*_contact_list)[i]->getRFConstraintVec(Uf_ieq_vec);
    _Uf_ieq_vec.segment(dim_accumul_uf, dim_new_uf) = Uf_ieq_vec;

    // Fr desired
    _Fr_des.segment(dim_accumul_rf, dim_new_rf) =                   //获取期望的足端力
      (*_contact_list)[i]->getRFDesired();
    dim_accumul_rf += dim_new_rf;                                         //更新累积反作用力维度和累积不等式约束维度
    dim_accumul_uf += dim_new_uf;
  }
  //pretty_print(_Fr_des, std::cout, "[WBIC] Fr des");
  // pretty_print(_Jc, std::cout, "[WBIC] Jc");
  // pretty_print(_JcDotQdot, std::cout, "[WBIC] JcDot Qdot");
  // pretty_print(_Uf, std::cout, "[WBIC] Uf");
}

template <typename T>
void WBIC<T>::_GetSolution(const DVec<T>& qddot, DVec<T>& cmd) {                        //获取WBIC的解，即关节角速度和关节扭矩。
  DVec<T> tot_tau;
  if (_dim_rf > 0) {
    _data->_Fr = DVec<T>(_dim_rf);
    // get Reaction forces
    for (size_t i(0); i < _dim_rf; ++i)
      _data->_Fr[i] = z[i + _dim_floating] + _Fr_des[i];
    tot_tau =
      WB::A_ * qddot + WB::cori_ + WB::grav_ - _Jc.transpose() * _data->_Fr;

  } else {
    tot_tau = WB::A_ * qddot + WB::cori_ + WB::grav_;                            
  }
  _data->_qddot = qddot;
  cmd = tot_tau.tail(WB::num_act_joint_);
  // cmd*= -1;
  // Torque check
  // DVec<T> delta_tau = DVec<T>::Zero(WB::num_qdot_);
  // for(size_t i(0); i<_dim_floating; ++i) delta_tau[i] = z[i];
  // pretty_print(tot_tau, std::cout, "tot tau original");
  // tot_tau += delta_tau;
  // pretty_print(tot_tau, std::cout, "tot tau result");
  // pretty_print(qddot, std::cout, "qddot");
  // pretty_print(_data->_Fr, std::cout, "Fr");
  // pretty_print(_Fr_des, std::cout, "Fr des");
}

template <typename T>
void WBIC<T>::_SetCost() {                                             //设置二次规划中的成本函数矩阵G和向量g0    
  // Set Cost                                                           //成本函数通常用于最小化某种性能指标，例如关节力矩的平方和，
  size_t idx_offset(0);                                                 //偏移量，用于计算成本函数矩阵G和向量g0的索引。
  for (size_t i(0); i < _dim_floating; ++i) {
    G[i + idx_offset][i + idx_offset] = _data->_W_floating[i];                      //设置浮动基座的权重，通常用于控制浮动基座的加速度，确保其稳定性和响应性
  }
  idx_offset += _dim_floating;                 
  for (size_t i(0); i < _dim_rf; ++i) {                                            //设置反作用力的权重，确保接触力满足物理约束
    G[i + idx_offset][i + idx_offset] = _data->_W_rf[i];                            //设置反作用力权重，通常用于控制反作用力的大小和方向，确保其满足物理约束
  }
  // pretty_print(_data->_W_floating, std::cout, "W floating");
  // pretty_print(_data->_W_rf, std::cout, "W rf");
}

template <typename T>
void WBIC<T>::UpdateSetting(const DMat<T>& A, const DMat<T>& Ainv,            //先执行这一步  ，获取单腿动力学方程的矩阵，更新
    const DVec<T>& cori, const DVec<T>& grav,
    void* extra_setting) {
  WB::A_ = A;                                                               //惯性矩阵
  WB::Ainv_ = Ainv;                                                          //惯性矩阵的逆矩阵
  WB::cori_ = cori;                                                           //科里奥利力向量，这里应该是计算好的，不然的话应该是矩阵*关节速度
  WB::grav_ = grav;                                                             //重力向量
  WB::b_updatesetting_ = true;

  (void)extra_setting;                          //忽略额外的设置
}

template <typename T>
void WBIC<T>::_SetOptimizationSize() {                       //确定优化变量的数据和约束条件的数量
  // Dimension
  _dim_rf = 0;                                                               //遍布所有的接触点，累积反作用力的维度和不等式约束的维度。
  _dim_Uf = 0;  // Dimension of inequality constraint          
  for (size_t i(0); i < (*_contact_list).size(); ++i) {
    _dim_rf += (*_contact_list)[i]->getDim();
    _dim_Uf += (*_contact_list)[i]->getDimRFConstraint();
  }

  _dim_opt = _dim_floating + _dim_rf;                     //设置优化向量维度，包括floating base和contact force 6+n
  _dim_eq_cstr = _dim_floating;                         //等式约束的维度6

  // Matrix Setting
  G.resize(0., _dim_opt, _dim_opt);
  g0.resize(0., _dim_opt);
  CE.resize(0., _dim_opt, _dim_eq_cstr);              
  ce0.resize(0., _dim_eq_cstr);

  // Eigen Matrix Setting
  _dyn_CE = DMat<T>::Zero(_dim_eq_cstr, _dim_opt);                           //初始化动态约束
  _dyn_ce0 = DVec<T>(_dim_eq_cstr);                                       //约束向量
  if (_dim_rf > 0) {                                                         //如果有反作用力，调整不等式约束矩阵CI为
    CI.resize(0., _dim_opt, _dim_Uf);                                   //不等式约束，                 
    ci0.resize(0., _dim_Uf);  
    _dyn_CI = DMat<T>::Zero(_dim_Uf, _dim_opt);                //动态不等式矩阵。初始化维度为，不等式维度*优化维度
    _dyn_ci0 = DVec<T>(_dim_Uf);                                  //动态不等式向量
 
    _Jc = DMat<T>(_dim_rf, WB::num_qdot_);                   //雅可比矩阵，反作用力维度*自由度数量18
    _JcDotQdot = DVec<T>(_dim_rf);                          //反作用力维度
    _Fr_des = DVec<T>(_dim_rf);                            

    _Uf = DMat<T>(_dim_Uf, _dim_rf);                    //不等式约束矩阵
    _Uf.setZero();                         
    _Uf_ieq_vec = DVec<T>(_dim_Uf);                     //不等式约束向量
  } else {                              //如果没有反作用力，就是一列向量
    CI.resize(0., _dim_opt, 1);
    ci0.resize(0., 1);
  }
}

template class WBIC<double>;
template class WBIC<float>;
