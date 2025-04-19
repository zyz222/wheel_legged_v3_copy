/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef XINGTIANROBOT_H
#define XINGTIANROBOT_H

#include "message/LowlevelState.h"
#include "common/xingtian_leg.h"
#include "common/spatial.h"
#include "common/SpatialInertia.h"



template <typename T>
struct FBModelState {                                  //浮动基状态
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Quat<T> bodyOrientation;       //机身姿态四元数
  Vec3<T> bodyPosition;
  SVec<T> bodyVelocity;  // body coordinates   机身坐标系的速度，6*1  角速度线速度
  DVec<T> q;                 // joint angles
  DVec<T> qd;                 // joint velocities

  /*!
   * Print the position of the body
   */
  // void print() const {
  //   printf("position: %.3f %.3f %.3f\n", bodyPosition[0], bodyPosition[1],
  //          bodyPosition[2]);
  // }
};
template struct FBModelState<double>;
template struct FBModelState<float>;
/*!
 * The result of running the articulated body algorithm on a rigid-body floating
 * base model
 */
template <typename T>
struct FBModelStateDerivative {                     //质心位置速度、关节加速度
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vec3<T> dBodyPosition;
  SVec<T> dBodyVelocity;
  DVec<T> qdd;
};
template struct FBModelStateDerivative<double>;
template struct FBModelStateDerivative<float>;

template <typename T>
class QuadrupedRobot{
public:
    QuadrupedRobot(){};
    ~QuadrupedRobot(){}

    Vec3<T> getX(LowlevelState<T> &state);   //获取足端位置
    Vec34<T> getVecXP(LowlevelState<T> &state);

    // Inverse Kinematics(Body/Hip Frame)
    Vec12<T> getQ(const Vec34<T> &feetPosition, FrameType frame);
    Vec12<T> getQd(const Vec34<T> &feetPosition, const Vec34<T> &feetVelocity,LowlevelState<T> &state, FrameType frame);
    Vec12<T> getTau(const Vec12<T> &q, const Vec34<T> feetForce);

    // Forward Kinematics
    Vec3<T> getFootPosition(LowlevelState<T> &state, int id, FrameType frame);
    Vec3<T> getFootVelocity(LowlevelState<T> &state, int id);
    Vec34<T> getFeet2BPositions(LowlevelState<T> &state, FrameType frame);
    Vec34<T> getFeet2BVelocities(LowlevelState<T> &state, FrameType frame);

    // LowlevelState<T> *_LowState;

    Mat3<T> getJaco(LowlevelState<T> &state, int legID);
    Vec2<T> getRobVelLimitX(){return _robVelLimitX;}
    Vec2<T> getRobVelLimitY(){return _robVelLimitY;}
    Vec2<T> getRobVelLimitYaw(){return _robVelLimitYaw;}
    Vec34<T> getFeetPosIdeal(){return _feetPosNormalStand;}
    T getRobMass(){return _mass;}
    Vec3<T> getPcb(){return _pcb;}
    Mat3<T> getRobInertial(){return _Ib;}   //获取机身的转动惯量
    void projectToReachableRange(Vec3<T> &pos);

    // 建立单腿拉格朗日动力学方程！
    // M*qdd + C*qd + G + J^T*F = Tau
    Mat3<T> calcInertialMatrix(const Vec3<T> &theta);    //计算惯性矩阵
    Mat3<T> calcCoriolisForce(const Vec3<T> &theta,const Vec3<T> &dtheta);    //计算科里奥利力
    Vec3<T> calcGravityForce(const Vec3<T> &theta);                 //计算重力
    Vec3<T> calcJointTorque(const Vec3<T> &theta,const Vec3<T> &dtheta,const Vec3<T> &ddtheta); //计算关节力矩
    Vec3<T> calcJointTorqueForce(const Vec3<T> &theta,const Vec3<T> &dtheta,const Vec3<T> &ddtheta,const Vec3<T> &F); //计算关节力矩

    Vec34<T> calcJointTorqueForceTotal(const Vec34<T> &theta,const Vec34<T> &dtheta,const Vec34<T> &ddtheta,const Vec34<T> &feet_force); //计算所有关节力矩，带接触力
    Vec34<T> calcJointTorqueTotal(const Vec34<T> &theta,const Vec34<T> &dtheta,const Vec34<T> &ddtheta); //计算所有关节力矩，无接触力
    Vec34<T> calcJointAcc(const Vec34<T> &theta,const Vec34<T> &dtheta,const Vec34<T> &ddtheta,const Vec34<T> &contact_torque,const Vec34<T> &_joint_torque); //计算关节加速度
    

    void setState(const FBModelState<T>& state_) {      //根据状态计算动力学模型
      _state = state_;                                //传递给模型内部成员变量
  
      _biasAccelerationsUpToDate = false;            //标记是否需要重新计算科氏力
      _compositeInertiasUpToDate = false;            //标记是否需要重新计算 惯性矩阵    为false都是需要重新计算
  
      resetCalculationFlags();                        //标记需要重新计算
    }
  
    /*!
     * Mark all previously calculated values as invalid
     */
    void resetCalculationFlags() {    //内部成员变量和标志位
      _articulatedBodiesUpToDate = false;            //标记关节和连杆的动力学信息是否是最新的
      _kinematicsUpToDate = false;                   //标记是否需要重新计算 关节运动学信息
      _forcePropagatorsUpToDate = false;                   //标记是否需要重新计算 刚体接触力信息
      _qddEffectsUpToDate = false;                          //标记是否需要重新计算 关节力矩信息
      _accelerationsUpToDate = false;                       //标记是否需要重新计算 加速度信息
    }
    
    bool _kinematicsUpToDate = false;
    bool _biasAccelerationsUpToDate = false;
    bool _accelerationsUpToDate = false;
    bool _compositeInertiasUpToDate = false;
    bool _articulatedBodiesUpToDate = false;
    bool _forcePropagatorsUpToDate = false;
    bool _qddEffectsUpToDate = false;

    void forwardKinematics();                  //正向运动学计算                                //对所有的连杆、关节、基座做前向运动学计算，更新连杆的位姿、速度等信息
    void biasAccelerations();                       //偏置加速度计算                         //计算关节的科氏力，更新关节的加速度等信息。计算连杆的偏置加速度
    void compositeInertias();                          //复合惯性矩阵                       //计算所有连杆的惯性矩阵，更新所有连杆的惯性矩阵等信息。复合惯性
    void forwardAccelerationKinematics();                    //正向加速度运动学                   //根据给定的关节力矩或者外力，使用前向-后向递推的方法，来求解关节加速度、浮动基加速度
    void contactJacobians();                                 //计算接触雅可比                   //计算各个接触点的雅可比矩阵
    



    FBModelState<T> _state;        //机器人状态相关的数据
    FBModelStateDerivative<T> _dState;   //状态加速度
      //线速度，角速度 加速度，角加速度，虚功，虚加速度，
    vectorAligned<SVec<T>> _v, _vrot, _a, _arot, _avp, _avprot, _c, _crot, _S,
        _Srot, _fvp, _fvprot, _ag, _agrot, _f, _frot;    

    vectorAligned<SVec<T>> _U, _Urot, _Utot, _pA, _pArot;
    vectorAligned<SVec<T>> _externalForces;     //外部力

    vectorAligned<SpatialInertia<T>> _IC;       //质心惯性矩阵
    //父连杆到子连杆的变换矩阵，绝对坐标系下的变换矩阵   存储内存对齐的容器，这里是存储六维矩阵
    vectorAligned<Mat6<T>> _Xup, _Xa, _Xuprot, _IA, _ChiUp;              //在空间刚体的变换矩阵结构中的树形遍历中用于从父连杆坐标系到子连杆坐标系的转换

    DMat<T> _H, _C;                                                             //质量、惯性矩阵，
    DVec<T> _Cqd, _G;                                     
    vectorAligned<D6Mat<T>> _J;                     
    vectorAligned<SVec<T>> _Jdqd;

    vectorAligned<D3Mat<T>> _Jc;              //接触雅可比矩阵
    vectorAligned<Vec3<T>> _Jcdqd;            //接触雅可比矩阵*关节角速度    //偏置加速度
    
    vector<JointType> _jointTypes;
    vector<CoordinateAxis> _jointAxes;
    vector<Mat6<T>, Eigen::aligned_allocator<Mat6<T>>> _Xtree, _Xrot;
    vector<SpatialInertia<T>, Eigen::aligned_allocator<SpatialInertia<T>>> _Ibody,     //每个都会对齐内存
        _Irot;
    vector<std::string> _bodyNames;

    size_t _nGroundContact = 0;     //存储接触点的索引  
    vector<size_t> _gcParent;
    vector<Vec3<T>> _gcLocation;
    vector<uint64_t> _footIndicesGC;

    vector<Vec3<T>> _pGC;
    vector<Vec3<T>> _vGC;

    vector<bool> _compute_contact_info;
  

    void addBase(const SpatialInertia<T>& inertia);                          //添加浮动基，连杆的惯性
    void addBase(T mass, const Vec3<T>& com, const Mat3<T>& I);              //添加浮动基
    int addGroundContactPoint(int bodyID, const Vec3<T>& location,              //添加地面接触点
                              bool isFoot = false);
    void addGroundContactBoxPoints(int bodyId, const Vec3<T>& dims);
    int addBody(const SpatialInertia<T>& inertia,                                 //添加连杆惯性属性，
                const SpatialInertia<T>& rotorInertia, T gearRatio, int parent,    //电机的惯性属性，传动比，父节点，关节类型，转动轴，变换矩阵，返回一个刚体的索引ID
                JointType jointType, CoordinateAxis jointAxis,
                const Mat6<T>& Xtree, const Mat6<T>& Xrot);
    int addBody(const MassProperties<T>& inertia,             
                const MassProperties<T>& rotorInertia, T gearRatio, int parent,
                JointType jointType, CoordinateAxis jointAxis,
                const Mat6<T>& Xtree, const Mat6<T>& Xrot);
    void check();                          //检查系统是否正确
    T totalRotorMass();                      //计算转子部分的质量
    T totalNonRotorMass();                   //计算非转子部分的质量

      /*!
    * Get vector of parents, where parents[i] is the parent body of body i
    * @return Vector of parents
    */
    const std::vector<int>& getParentVector() { return _parents; }

    
    /*!
    * Get vector of body spatial inertias
    * @return Vector of body spatial inertias     //const表示返回的引用数据不能被修改   &表示避免返回整个向量拷贝
    */
    const std::vector<SpatialInertia<T>,       //标志空间惯性类的结构体
                      Eigen::aligned_allocator<SpatialInertia<T>>>&         //这么定义是为了保证内存对齐，
    getBodyInertiaVector() {                           //返回机身的空间惯性向量
      return _Ibody;
    }

    /*!
    * Get vector of rotor spatial inertias
    * @return Vector of rotor spatial inertias
    */
    const std::vector<SpatialInertia<T>,
                      Eigen::aligned_allocator<SpatialInertia<T>>>&
    getRotorInertiaVector() {              //返回电机的空间惯性向量
      return _Irot;
    }

    /*!
    * Set the gravity
    */
    void setGravity(Vec3<T>& g) { _gravity = g; }     //设置重力向量


    /*
    * Set the flag to enable computing contact info for a given contact point
    * @param gc_index : index of contact point
    * @param flag : enable/disable contact calculation
    */
  void setContactComputeFlag(size_t gc_index, bool flag) {              //启用或者禁用特定接触点gc_index的接触信息计算。
    _compute_contact_info[gc_index] = flag;
  }

  DMat<T> invContactInertia(const int gc_index,                                //用于计算接触点的惯性矩阵的逆
                            const D6Mat<T>& force_directions);
  T invContactInertia(const int gc_index, const Vec3<T>& force_ics_at_contact);

  T applyTestForce(const int gc_index, const Vec3<T>& force_ics_at_contact,      //在指定点添加一个测试力
                    FBModelStateDerivative<T>& dstate_out);

  T applyTestForce(const int gc_index, const Vec3<T>& force_ics_at_contact,
                    DVec<T>& dstate_out);

  void addDynamicsVars(int count);

  void resizeSystemMatricies();

  /*!
  * Update the state derivative of the simulator, invalidating previous results.
  * @param dState : the new state derivative
  */
  void setDState(const FBModelStateDerivative<T>& dState) {                //设置系统的加速度、关节加速度等导数状态
    _dState = dState;
    _accelerationsUpToDate = false;                                         //标记是否需要重新计算 加速度信息
  }


  Vec3<T> getPosition(const int link_idx, const Vec3<T> & local_pos);       //获取连杆的位置，世界坐标系下，或者某个局部点的位置
  Vec3<T> getPosition(const int link_idx);                                   //获取连杆的位置，世界坐标系下


  Mat3<T> getOrientation(const int link_idx);  //获取连杆的旋转矩阵，世界坐标系下，从该连杆坐标系转换到世界坐标系下
  Vec3<T> getLinearVelocity(const int link_idx, const Vec3<T>& point);  //获取连杆的局部坐标系下某个点的速度，世界坐标系下
  Vec3<T> getLinearVelocity(const int link_idx);                          //获取连杆的速度，世界坐标系下

  Vec3<T> getLinearAcceleration(const int link_idx, const Vec3<T>& point);   //获取连杆的局部坐标系下某个点的加速度，世界坐标系下     
  Vec3<T> getLinearAcceleration(const int link_idx);

  Vec3<T> getAngularVelocity(const int link_idx);
  Vec3<T> getAngularAcceleration(const int link_idx);


  DVec<T> generalizedGravityForce();                       //分别返回系统的广义重力力和广义科氏力向量。维度同（Dof）
  DVec<T> generalizedCoriolisForce();                           
  DMat<T> massMatrix();                 //返回整机（浮动基+连杆）的质量矩阵H，dof*dof。6个浮动基自由度+n个关节自由度
  DVec<T> inverseDynamics(const FBModelStateDerivative<T>& dState);                 //给定加速度来求解关节力矩/广义力
  void runABA(const DVec<T>& tau, FBModelStateDerivative<T>& dstate);                 //ABA算法，给定关节力矩，求解关节加速度，浮动基加速度，以及各个接触点的外力
                                                                                          //这是在在线模拟，多体仿真中常用的正向求解。
  size_t _nDof = 0; 
  Vec3<T> _gravity;
  vector<int> _parents;
  vector<T> _gearRatios;
  vector<T> _d, _u;

  /*
  * Get the mass matrix for the system
    */
  const DMat<T>& getMassMatrix() const { return _H; }

  /*!
    * Get the gravity term (generalized forces)
    */
  const DVec<T>& getGravityForce() const { return _G; }

  /*!
    * Get the coriolis term (generalized forces)
    */
  const DVec<T>& getCoriolisForce() const { return _Cqd; }

  void updateArticulatedBodies();
  void updateForcePropagators();
  void updateQddEffects();

  /*!
    * Set all external forces to zero
    */
  void resetExternalForces() {                                               //设置外部力为0
    for (size_t i = 0; i < _nDof; i++) {
      _externalForces[i] = SVec<T>::Zero();
    }
  }

  DMat<T> _qdd_from_base_accel;                       //基座加速度对关节加速度的影响矩阵
  DMat<T> _qdd_from_subqdd;
  Eigen::ColPivHouseholderQR<Mat6<T>> _invIA5;

protected:
    QuadrupedLeg<T>* _Legs[4];
    Vec2<T> _robVelLimitX;
    Vec2<T> _robVelLimitY;
    Vec2<T> _robVelLimitYaw;
    Vec34<T> _feetPosNormalStand;
    T _mass;   //简化模型质量
    Vec3<T> _pcb;
    Mat3<T> _Ib;      //简化模型的转动惯量

   
    // int _nDof = xingtian::dim_config;   //18个自由度

    Mat3<T> M;  //质量惯性矩阵
    Mat3<T> C;  //科里奥利矩阵
    Vec3<T> G;  //重力矩阵
    
    Vec3<T> F; //接触力
    
    T hip_mass;    //大腿质量
    T knee_mass;    //小腿质量
    T wheel_mass;    //车轮质量
    T L1,L2,Lc1,Lc2;  //连杆长度以及重心位置
    T q1,q2,q3;
    T qd1,qd2,qd3;     //关节速度
    T qdd1,qdd2,qdd3;   //关节加速度
    T Iyy1,Iyy2,Iyy3;  //大腿、小腿、车轮转动惯量，应该是绕y轴转动
    T m11,m12,m13;
    T m21,m22,m23;
    T m31,m32,m33;
    T c11,c12,c13;
    T c21,c22,c23;
    T c31,c32,c33;
    T g1,g2,g3;
    T _g;  //重力加速度


};


template <typename T>
class xingtianRobot : public QuadrupedRobot<T>{
public:
    xingtianRobot();
    ~xingtianRobot(){};
};

#endif  // XINGTIANROBOT_H