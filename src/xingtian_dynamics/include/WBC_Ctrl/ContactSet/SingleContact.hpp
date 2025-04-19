#ifndef Cheetah_SINGLE_CONTACT
#define Cheetah_SINGLE_CONTACT

#include <common/xingtianrobot.h>
// #include <Dynamics/Quadruped.h>
#include <WBC/ContactSpec.hpp>

template <typename T>
class SingleContact : public ContactSpec<T> {
 public:
  SingleContact(const QuadrupedRobot<T>* robot, int contact_pt);
  virtual ~SingleContact();

  void setMaxFz(T max_fz) { _max_Fz = max_fz; }   //设置最大力的值

 protected:
  T _max_Fz;                              //最大的法向力
  int _contact_pt;                           //接触点的编号，四条腿的编号
  int _dim_U;                               //不等式向量的维度

  virtual bool _UpdateJc();
  virtual bool _UpdateJcDotQdot();
  virtual bool _UpdateUf();
  virtual bool _UpdateInequalityVector();

  const QuadrupedRobot<T>* robot_sys_;
};

#endif
