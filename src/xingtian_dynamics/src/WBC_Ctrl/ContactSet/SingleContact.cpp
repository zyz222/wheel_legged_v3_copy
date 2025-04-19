#include "WBC_Ctrl/ContactSet/SingleContact.hpp"
#include <Utilities/Utilities_print.h>

// [ Fx, Fy, Fz ]
template <typename T>    //pt是接触腿的编号！！！
SingleContact<T>::SingleContact(const QuadrupedRobot<T>* robot, int pt)     //定义机器人在特定接触点的力约束，包括摩擦锥，和法向力的上下限。
    : ContactSpec<T>(3), _max_Fz(250.), _contact_pt(pt), _dim_U(6) {             //初始化最大接触力为1500N,接触权重
  Contact::idx_Fz_ = 2;                             
  robot_sys_ = robot;
  Contact::Jc_ = DMat<T>(Contact::dim_contact_, xingtian::dim_config);    //定义雅可比矩阵为6*12   ,接触雅可比矩阵
  Contact::JcDotQdot_ = DVec<T>::Zero(Contact::dim_contact_);          //初始化为0向量   3维
  Contact::Uf_ = DMat<T>::Zero(_dim_U, Contact::dim_contact_);       //6维*3

  T mu(0.5);                  //摩擦系数
//力约束矩阵
  Contact::Uf_(0, 2) = 1.;         //【0 0 1】  fz方向力约束
  
  Contact::Uf_(1, 0) = 1.;
  Contact::Uf_(1, 2) = mu;
  Contact::Uf_(2, 0) = -1.;
  Contact::Uf_(2, 2) = mu;      //fx方向力约束

  Contact::Uf_(3, 1) = 1.;
  Contact::Uf_(3, 2) = mu;
  Contact::Uf_(4, 1) = -1.;
  Contact::Uf_(4, 2) = mu;            //fy方向力约束

  // Upper bound of normal force
  Contact::Uf_(5, 2) = -1.;           //fz方向力约束
}

template <typename T>
SingleContact<T>::~SingleContact() {}

template <typename T>
bool SingleContact<T>::_UpdateJc() {     //更新约束雅可比矩阵
  Contact::Jc_ = robot_sys_->_Jc[_contact_pt];         //获取接触点的雅可比矩阵

  // Quat<T> quat = robot_sys_->_state.bodyOrientation;
  // Mat3<T> Rot = ori::quaternionToRotationMatrix(quat);
  // Contact::Jc_.block(0,3, 3,3) = Rot*Contact::Jc_.block(0,3,3,3);

  // Contact::Jc_.block(0,0, 3,3) = Rot.transpose()*Contact::Jc_.block(0,0,3,3);
  // pretty_print(Rot, std::cout, "body ori");
  // pretty_print(Contact::Jc_, std::cout, "Jc");
  return true;
}

template <typename T>
bool SingleContact<T>::_UpdateJcDotQdot() {                             //更新雅可比矩阵的时间导数乘以关节速度向量，这个向量通常表示加速度效应或者惯性效应。
  Contact::JcDotQdot_ = robot_sys_->_Jcdqd[_contact_pt];
  // pretty_print(Contact::JcDotQdot_, std::cout, "JcDotQdot");
  return true;
}

template <typename T>
bool SingleContact<T>::_UpdateUf() {                       //更新约束矩阵
  return true;
}

template <typename T>
bool SingleContact<T>::_UpdateInequalityVector() {         //更新约束向量
  Contact::ieq_vec_ = DVec<T>::Zero(_dim_U);
  Contact::ieq_vec_[5] = -_max_Fz;                //足端施加的力
  return true;
}

template class SingleContact<double>;
template class SingleContact<float>;
