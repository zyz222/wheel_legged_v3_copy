/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef ESTIMATOR_H
#define ESTIMATOR_H
#pragma once
#include <vector>
#include "common/xingtianrobot.h"
#include "common/LowPassFilter.h"
#include "Gait/WaveGenerator.h"
#include "message/LowlevelState.h"
#include "string"
#include "control/CtrlComponents.h"

template <typename T>
struct StateEstimate {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vec4<T> contactEstimate;
    Vec3<T> position;
    Vec3<T> vBody;
    Quat<T> orientation;
    Vec3<T> omegaBody;
    RotMat<T> rBody;     //旋转矩阵
    Vec3<T> rpy;
    Vec3<T> omegaWorld;
    Vec3<T> vWorld;
    Vec3<T> aBody, aWorld;
};

// StateEstimate<float> stateEstimate;

template <typename T>
class Estimator{
public:
    Estimator(QuadrupedRobot<T> *robotModel, LowlevelState<T>* lowState, Vec4<T> *contact, Vec4<T> *phase, T dt);
    Estimator(QuadrupedRobot<T> *robotModel, LowlevelState<T>* lowState, Vec4<T> *contact, Vec4<T> *phase, T dt, Vec18<T> Qdig, std::string testName);
    ~Estimator();
    // Vec3<float>  cheater_getPosition_f(){
    //     Eigen::Vector3d vec_d = Eigen::Map<Eigen::Vector3d>(_lowState->xingtian_state.position);
    //     return vec_d.cast<float>();}                             //世界坐标下的位置
    // Vec3<float>  cheater_getVelocity_f(){
    //     Eigen::Vector3d vec_d = Eigen::Map<Eigen::Vector3d>(_lowState->xingtian_state.vBody);
    //     return vec_d.cast<float>();}    //世界坐标系下的速度
    // Vec3<T>  cheater_getPosition(){ return Eigen::Map<Eigen::Vector3d>(_lowState->xingtian_state.position);}  
    // Eigen::Map<Eigen::Vector3f>                           //世界坐标下的位置
    Vec3<T>  cheater_getPosition(){ return Eigen::Map<Vec3<T>>(_lowState->xingtian_state.position);}  
    Vec3<T>  cheater_getVelocity(){ return Eigen::Map<Vec3<T>>(_lowState->xingtian_state.vBody);}    //世界坐标系下的速度
    Vec3<T>  cheater_getFootPos(int i){return this-> cheater_getPosition()+  _lowState->getRotMat() * _robModel->getFootPosition(*_lowState, i, FrameType::BODY); }                                         //足端位置
    Vec34<T> cheater_getFeetPos();   
    Vec34<T> cheater_getFeetVel();         //足端速度
    Vec34<T> cheater_getPosFeet2BGlobal(); 

    Vec3<T>  getPosition();    //世界坐标下的位置
    Vec3<T>  getVelocity();    //世界坐标系下的速度
    Vec3<T>  getFootPos(int i);  //足端位置
    Vec34<T> getFeetPos();   
    Vec34<T> getFeetVel();       //足端速度
    Vec34<T> getPosFeet2BGlobal();   //世界坐标系下相对于机身的足端位置
    //添加接触相位估计
    Vec4<T> getContactEstimator(){ return *_contact;}
    //添加返回姿态信息
    Quat<T> getQuat() const {return _lowState->imu.getQuat();}    

    // Quat<T> getQuat_f() const {
    //     Eigen::Vector4d vec_d = _lowState->imu.getQuat();
    //     return vec_d.cast<float>();} 

    // Quat<T> getQuatworld()const {return getRotMat() * _lowState->imu.getQuat();}   
    Vec3<T> getRPY(){return _lowState->imu.getRPY();}
    // Vec3<T> getRPY_f(){
    //     Eigen::Vector3d vec_d  = _lowState->imu.getRPY();
    //     return vec_d.cast<float>();}

    //返回角速度信息Body
    Vec3<T> getRPYworld(){return getRotMat() * _lowState->imu.getRPY();}
    Vec3<T> getomega()const {return _lowState->imu.getGyro();}

    // Vec3<T> getomega_f()const {
    //     Eigen::Vector3d vec_d = _lowState->imu.getGyro();
    //     return vec_d.cast<float>();}

    Vec3<T> getomegaworld()const {return getRotMat() * _lowState->imu.getGyro();}
    // Vec3<T> getomegaworld_f()const {
    //     Eigen::Vector3d vec_d = getRotMat() * _lowState->imu.getGyro();
    //     return vec_d.cast<float>();}
    //返回机身旋转矩阵
    RotMat<T> getRotMat()const {return _lowState->imu.getRotMat();}

    // RotMat<T> getRotMat_f()const {
    //     Eigen::Matrix3d vec_d = _lowState->imu.getRotMat();
    //     return vec_d.cast<float>();}
    //返回世界坐标系下的加速度
    Vec3<T> getAccGlobal(){return _lowState->getAccGlobal();}
    // Vec3<T> getAccGlobal_f(){
    //     Eigen::Vector3d vec_d = _lowState->getAccGlobal();
    //     return vec_d.cast<float>();}
    //返回机身坐标系下的加速度
    Vec3<T> getAcc2B(){return _lowState->getAcc();}
    
    // Vec3<T> getAcc2B_f(){
    //     Eigen::Vector3d vec_d = _lowState->getAcc();
    //     return vec_d.cast<float>();}
    //返回机身坐标下的速度
    Vec3<T> getVelocity2B(){
        return getRotMat().transpose()*cheater_getVelocity();}
    // Vec3<T> getVelocity2B_f(){
    //     Eigen::Vector3d vec_d = getRotMat().transpose()*cheater_getVelocity();
    //     return vec_d.cast<float>();}
    
    void run();
    StateEstimate<T> stateEstimate;   //状态估计的结构体容器

private:
    void _initSystem();
    // Linear System
    Vec18<T>  _xhat;            // The state of estimator, position(3)+velocity(3)+feet position(3x4)
    Vec3<T> _u;                                        // The input of estimator
    Eigen::Matrix<T, 28,  1> _y;               // The measurement value of output y
    Eigen::Matrix<T, 28,  1> _yhat;            // The prediction of output y
    Eigen::Matrix<T, 18, 18> _A;               // The transtion matrix of estimator
    Eigen::Matrix<T, 18, 3>  _B;               // The input matrix
    Eigen::Matrix<T, 28, 18> _C;               // The output matrix
    // Covariance Matrix
    Eigen::Matrix<T, 18, 18> _P;               // Prediction covariance
    Eigen::Matrix<T, 18, 18> _Ppriori;         // Priori prediction covariance
    Eigen::Matrix<T, 18, 18> _Q;               // Dynamic simulation covariance
    Eigen::Matrix<T, 28, 28> _R;               // Measurement covariance
    Eigen::Matrix<T, 18, 18> _QInit;           // Initial value of Dynamic simulation covariance
    Eigen::Matrix<T, 28, 28> _RInit;           // Initial value of Measurement covariance
    Vec18<T> _Qdig;                                    // adjustable process noise covariance
    Mat3<T> _Cu;                                       // The covariance of system input u
    // Output Measurement
    Eigen::Matrix<T, 12, 1>  _feetPos2Body;    // The feet positions to body, in the global coordinate
    Eigen::Matrix<T, 12, 1>  _feetVel2Body;    // The feet velocity to body, in the global coordinate
    Eigen::Matrix<T,  4, 1>  _feetH;           // The Height of each foot, in the global coordinate
    Eigen::Matrix<T, 28, 28> _S;               // _S = C*P*C.T + R
    Eigen::PartialPivLU<Eigen::Matrix<T, 28, 28>> _Slu;    // _S.lu()
    Eigen::Matrix<T, 28,  1> _Sy;              // _Sy = _S.inv() * (y - yhat)
    Eigen::Matrix<T, 28, 18> _Sc;              // _Sc = _S.inv() * C
    Eigen::Matrix<T, 28, 28> _SR;              // _SR = _S.inv() * R
    Eigen::Matrix<T, 28, 18> _STC;             // _STC = (_S.transpose()).inv() * C
    Eigen::Matrix<T, 18, 18> _IKC;             // _IKC = I - KC

    RotMat<T> _rotMatB2G;                              // Rotate Matrix: from body to global
    Vec3<T> _g;
    Vec34<T> _feetPosGlobalKine, _feetVelGlobalKine;

    LowlevelState<T>* _lowState;
    QuadrupedRobot<T> *_robModel;
    Vec4<T> *_phase;
    Vec4<T> *_contact;
    T _dt;
    T _trust;
    T _largeVariance;

    // Low pass filters
    LPFilter<T> *_vxFilter, *_vyFilter, *_vzFilter;

    // Tuning
    AvgCov<T> *_RCheck;
    AvgCov<T> *_uCheck;
    std::string _estName;

};

#endif  // ESTIMATOR_H