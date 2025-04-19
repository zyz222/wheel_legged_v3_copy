/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
// 获取关节电机的状态和机身的姿态
#ifndef LOWLEVELSTATE_HPP
#define LOWLEVELSTATE_HPP

#include <iostream>
#include "common/cppTypes.h"
#include "common/mathTools.h"
#include "interface/CmdPanel.h"
#include "common/enumClass.h"
#include "cmath"
// #include "common/xingtianrobot.h"
struct MotorState
{
	unsigned int mode;
    float q;
    float dq;
    float ddq;
    float tauEst;

    MotorState(){
        q = 0;
        dq = 0;
        ddq = 0;
        tauEst = 0;
    }
};

template <typename T>
struct xingtian_State {
  T orientation[4];
  T position[3];
  T omegaBody[3];
  T vBody[3];
  T acceleration[3];
  T wheel_force[12];
};
template struct xingtian_State<double>;
template struct xingtian_State<float>;

template <typename T>
struct IMU
{
    T quaternion[4];    // w, x, y, z
    T gyroscope[3];
    T accelerometer[3];
    T linearvelocity[3];
    IMU(){
        for(int i = 0; i < 3; i++){
            quaternion[i] = 0;
            gyroscope[i] = 0;
            accelerometer[i] = 0;
            linearvelocity[i] = 0;
        }
        quaternion[3] = 0;
    }

    RotMat<T> getRotMat(){
        Quat<T> quat;
        quat << quaternion[0], quaternion[1], quaternion[2], quaternion[3];
        return quatToRotMat(quat);       //这个没问题！！
    }
    Vec3<T> getRPY(){
        Vec3<T> rpy;
        RotMat<T> R = getRotMat();
        rpy(0) = atan2(R(2,1),R(2,2));
        rpy(1) = asin(-R(2,0));
        rpy(2) = atan2(R(1,0),R(0,0));
        return rpy;
    }
    Vec3<T> getAcc(){
        Vec3<T> acc;
        acc << accelerometer[0], accelerometer[1], accelerometer[2];
        return acc;
    }
    Vec3<T> getVelocity(){
        Vec3<T> velocity;
        velocity << linearvelocity[0], linearvelocity[1], linearvelocity[2];
        return velocity;
    }
    Vec3<T> getGyro(){
        Vec3<T> gyro;
        gyro << gyroscope[0], gyroscope[1], gyroscope[2];
        return gyro;
    }

    Quat<T> getQuat(){
        Quat<T> q;
        q << quaternion[0], quaternion[1], quaternion[2], quaternion[3];
        return q;
    }
};
template struct IMU<double>;
template struct IMU<float>;

template <typename T>
struct LowlevelState
{
    IMU<T> imu;
    MotorState motorState[12];
    UserCommand userCmd;
    UserValue userValue;
    xingtian_State<T> xingtian_state;
    Vec34<T> getQ()const {
        Vec34<T> qLegs;      //三行四列矩阵
        for(int i(0); i < 4; ++i){
            qLegs.col(i)(0) = motorState[3*i    ].q;
            qLegs.col(i)(1) = motorState[3*i + 1].q;
            qLegs.col(i)(2) = motorState[3*i + 2].q;
        }
        return qLegs;
    }

    Vec34<T> getQd()const {
        Vec34<T> qdLegs;
        for(int i(0); i < 4; ++i){
            qdLegs.col(i)(0) = motorState[3*i    ].dq;
            qdLegs.col(i)(1) = motorState[3*i + 1].dq;
            qdLegs.col(i)(2) = motorState[3*i + 2].dq;
        }
        return qdLegs;
    }
    Vec34<T> gettau(){
        Vec34<T> tauLegs;
        for(int i(0); i < 4; ++i){
            tauLegs.col(i)(0) = motorState[3*i    ].tauEst;
            tauLegs.col(i)(1) = motorState[3*i + 1].tauEst;
            tauLegs.col(i)(2) = motorState[3*i + 2].tauEst;
        }
        return tauLegs;
    }

    RotMat<T> getRotMat(){
        return imu.getRotMat();
    }

    Vec3<T> getAcc(){
        return imu.getAcc();
    }

    Vec3<T> getGyro(){
        return imu.getGyro();
    }

    Vec3<T> getRPY(){

        return imu.getRPY();
    }
    Vec3<T> getAccGlobal(){
        return getRotMat() * getAcc();
    }

    Vec3<T> getGyroGlobal(){
        return getRotMat() * getGyro();
    }

    T getYaw(){
        return rotMatToRPY(getRotMat())(2);
    }

    T getDYaw(){
        return getGyroGlobal()(2);
    }

    void setQ(Vec12<T> q){
        for(int i(0); i<12; ++i){
            motorState[i].q = q(i);
        }
    }
};
template struct LowlevelState<double>;
template struct LowlevelState<float>;
#endif  //LOWLEVELSTATE_HPP