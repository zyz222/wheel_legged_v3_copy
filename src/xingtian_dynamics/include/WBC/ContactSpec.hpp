// 这个也不怎么需要改

#ifndef D5381BED_8583_479E_9C77_305F0524A30C
#define D5381BED_8583_479E_9C77_305F0524A30C
#ifndef CONTACT_SPEC
#define CONTACT_SPEC

#include "common/cppTypes.h"
#include "common/Quadruped.h"
#define Contact ContactSpec<T>

template <typename T>
class ContactSpec {                             //管理接触规范的抽象类，用于机器人与环境接触时的力和运动约束
 public:
  ContactSpec(size_t dim) : dim_contact_(dim), b_set_contact_(false) {
    idx_Fz_ = dim - 1;  // because normally (tau_x,y,z , linear_x,y,z)        //最后一个方向代表垂直方向的力，Z方向索引
    Fr_des_ = DVec<T>::Zero(dim);                                            //初始化力向量为0
  }                                   
  virtual ~ContactSpec() {}

  size_t getDim() const { return dim_contact_; }                  //获取接触力维度
  size_t getDimRFConstraint() const { return Uf_.rows(); }   //获取接触矩阵的维度。不等式矩阵约束，用于定义接触力的约束条件，返回接触力的大小
  size_t getFzIndex() const { return idx_Fz_; }                        //返回Z轴力的索引

  void getContactJacobian(DMat<T>& Jc) { Jc = Jc_; }                    //返回接触雅可比矩阵
  void getJcDotQdot(DVec<T>& JcDotQdot) { JcDotQdot = JcDotQdot_; }     //返回接触雅可比矩阵的时间导数与关节速度向量的乘积
  void UnsetContact() { b_set_contact_ = false; }                  //将接触状态标记为未设置

  void getRFConstraintMtx(DMat<T>& Uf) { Uf = Uf_; }                       //获取不等式约束矩阵和约束向量
  void getRFConstraintVec(DVec<T>& ieq_vec) { ieq_vec = ieq_vec_; }
  const DVec<T>& getRFDesired() { return Fr_des_; }                   //获取期望力
  void setRFDesired(const DVec<T>& Fr_des) { Fr_des_ = Fr_des; }               //返回期望力

  bool UpdateContactSpec() {                                         //更新接触规范
    _UpdateJc();       
    _UpdateJcDotQdot();
    _UpdateUf();
    _UpdateInequalityVector();
    b_set_contact_ = true;
    return true;
  }

 protected:
  virtual bool _UpdateJc() = 0;
  virtual bool _UpdateJcDotQdot() = 0;
  virtual bool _UpdateUf() = 0;
  virtual bool _UpdateInequalityVector() = 0;

  int idx_Fz_;
  DMat<T> Uf_;                        //不等式约束矩阵，用于定义接触力的约束条件
  DVec<T> ieq_vec_;                  //不等式约束向量，用于定义接触力的上下界
  DVec<T> Fr_des_;                 //期望力的接触向量

  DMat<T> Jc_;                      //接触力雅克比矩阵
  DVec<T> JcDotQdot_;             //接触力矩阵的时间导数与关节速度向量的乘积，表示动态效应
  size_t dim_contact_;             //接触力维度
  bool b_set_contact_;              //是否设置接触力
};
#endif


#endif /* D5381BED_8583_479E_9C77_305F0524A30C */
