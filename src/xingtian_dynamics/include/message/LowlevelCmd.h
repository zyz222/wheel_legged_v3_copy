/**********************************************************************
 Copyright (c) CIEM-2024-ZYZ. All rights reserved.
***********************************************************************/
#ifndef LOWLEVELCMD_H
#define LOWLEVELCMD_H

#include "common/cppTypes.h"
#include "common/mathTools.h"
#include <cmath>
struct MotorCmd{
    unsigned int mode;
    float q;
    float dq;
    float tau;
    float Kp;
    float Kd;

    MotorCmd(){
        mode = 0;
        q = 0;
        dq = 0;
        tau = 0;
        Kp = 0;
        Kd = 0;
    }
};


template <typename T>
struct LegControllerCommand {             //发布腿部控制器命令
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LegControllerCommand() { zero(); }

  void zero()
  {
    tauFeedForward = Vec3<T>::Zero();
    forceFeedForward = Vec3<T>::Zero();
    qDes = Vec3<T>::Zero();
    qdDes = Vec3<T>::Zero();
    pDes = Vec3<T>::Zero();
    vDes = Vec3<T>::Zero();
    kpCartesian = Mat3<T>::Zero();
    kdCartesian = Mat3<T>::Zero();
    kpJoint = Mat3<T>::Zero();
    kdJoint = Mat3<T>::Zero();
  }

  Vec3<T> tauFeedForward, forceFeedForward, qDes, qdDes, pDes, vDes;
  Mat3<T> kpCartesian, kdCartesian, kpJoint, kdJoint;   //前两个是任务Kp.kd,后两个是关节KpKd
};
template struct LegControllerCommand<double>;
template struct LegControllerCommand<float>;
template <typename T>
struct LegControllerData {                                           //更新腿部控制器数据
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LegControllerData() { zero(); }

//   void setQuadruped(Quadruped<T>& quad) { quadruped = &quad; }

  void zero(){
    q = Vec3<T>::Zero();
    qd = Vec3<T>::Zero();
    p = Vec3<T>::Zero();                                             //腿部位置
    v = Vec3<T>::Zero();                                             //腿部速度
    J = Mat3<T>::Zero();                                             //腿部雅克比
    tauEstimate = Vec3<T>::Zero();
    }

  Vec3<T> q, qd, p, v;
  Mat3<T> J;
  Vec3<T> tauEstimate;
//   Quadruped<T>* quadruped;
};
template struct LegControllerData<double>;
template struct LegControllerData<float>;



template <typename T>
struct Task_KpKd
{
    T Kp_body[3];
    T Kd_body[3];
    T Kp_ori[3];
    T Kd_ori[3];
    T Kp_foot[12];
    T Kd_foot[12];
    T Kp_joint[3];
    T Kd_joint[3];
    Task_KpKd(){
        // Kp_body.setZero();
        // Kd_body.setZero();
        // Kp_ori.setZero();
        // Kd_ori.setZero();
        // Kp_foot.setZero();
        // Kd_foot.setZero();
        // Kp_joint.setZero();
        // Kd_joint.setZero();
        std::fill(std::begin(Kp_body),std::end(Kp_body),500);
        std::fill(std::begin(Kd_body),std::end(Kd_body),5);
        std::fill(std::begin(Kp_ori),std::end(Kp_ori),50000);
        std::fill(std::begin(Kd_ori),std::end(Kd_ori),5);
        std::fill(std::begin(Kp_foot),std::end(Kp_foot),5000);
        std::fill(std::begin(Kd_foot),std::end(Kd_foot),5);
        std::fill(std::begin(Kp_joint),std::end(Kp_joint),500);
        std::fill(std::begin(Kd_joint),std::end(Kd_joint),5);
        
    }
    /* data */
};
template struct Task_KpKd<double>;
template struct Task_KpKd<float>;

template <typename T>
struct LowlevelCmd{
    MotorCmd motorCmd[12];
    Task_KpKd<T> task_kpkd;
    LegControllerCommand<float> commands[4];     //4条腿的控制指令

    void setQ(Vec12<T> q){
        for(int i(0); i<12; ++i){
            motorCmd[i].q = q(i);
        }
    }
    void setQ(int legID, Vec3<T> qi){
        motorCmd[legID*3+0].q = qi(0);
        motorCmd[legID*3+1].q = qi(1);
        motorCmd[legID*3+2].q = qi(2);
    }
    void setQd(Vec12<T> qd){
        for(int i(0); i<12; ++i){
            motorCmd[i].dq = qd(i);
        }
    }
    void setQd(int legID, Vec3<T> qdi){
        motorCmd[legID*3+0].dq = qdi(0);
        motorCmd[legID*3+1].dq = qdi(1);
        motorCmd[legID*3+2].dq = qdi(2);
    }
    // 实机部署时改为20
    void setTau(Vec12<T> tau, Vec2<T> torqueLimit = Vec2<T>(-30, 30)){
        for(int i(0); i<12; ++i){
            if(std::isnan(tau(i))){
                printf("[ERROR] The setTau function meets Nan\n");
                tau(i) = 0;
            }
            motorCmd[i].tau = saturation(tau(i), torqueLimit);
        }
    }
    void setZeroDq(int legID){
        motorCmd[legID*3+0].dq = 0;
        motorCmd[legID*3+1].dq = 0;
        motorCmd[legID*3+2].dq = 0;
    }
    void setZeroDq(){
        for(int i(0); i<4; ++i){
            setZeroDq(i);
        }
    }
    void setZeroTau(int legID){
        motorCmd[legID*3+0].tau = 0;
        motorCmd[legID*3+1].tau = 0;
        motorCmd[legID*3+2].tau = 0;
    }
    void setSimStanceGain(int legID){
        motorCmd[legID*3+0].mode = 1;    //hip
        motorCmd[legID*3+0].Kp = 100;
        motorCmd[legID*3+0].Kd = 3;    
        motorCmd[legID*3+1].mode = 1;      //knee
        motorCmd[legID*3+1].Kp = 100;
        motorCmd[legID*3+1].Kd = 3;
        // motorCmd[legID*3+2].mode = 1;      //车轮
        // motorCmd[legID*3+2].Kp = 2;
        // motorCmd[legID*3+2].Kd = 1.5;
    }
    void setSimFreeStanceGain(int legID){
        motorCmd[legID*3+0].mode = 1;    //hip
        motorCmd[legID*3+0].Kp = 150;
        motorCmd[legID*3+0].Kd = 3;    
        motorCmd[legID*3+1].mode = 1;      //knee
        motorCmd[legID*3+1].Kp = 150;
        motorCmd[legID*3+1].Kd = 3;

    }
    void setRealStanceGain(int legID){
        motorCmd[legID*3+0].mode = 10;
        motorCmd[legID*3+0].Kp = 60;
        motorCmd[legID*3+0].Kd = 5;
        motorCmd[legID*3+1].mode = 10;
        motorCmd[legID*3+1].Kp = 40;
        motorCmd[legID*3+1].Kd = 4;

    }
    void setZeroGain(int legID){
        motorCmd[legID*3+0].mode = 1;
        motorCmd[legID*3+0].Kp = 0;
        motorCmd[legID*3+0].Kd = 0;
        motorCmd[legID*3+1].mode = 1;
        motorCmd[legID*3+1].Kp = 0;
        motorCmd[legID*3+1].Kd = 0;
        // motorCmd[legID*3+2].mode = 1;
        // motorCmd[legID*3+2].Kp = 0;
        // motorCmd[legID*3+2].Kd = 0;
    }
    void setZeroGain(){
        for(int i(0); i<4; ++i){
            setZeroGain(i);
        }
    }
    // 平衡控制器的增益！！！！！
    void setStableGain(int legID){
        motorCmd[legID*3+0].mode = 1;
        motorCmd[legID*3+0].Kp = 20;
        motorCmd[legID*3+0].Kd = 3;
        motorCmd[legID*3+1].mode = 1;
        motorCmd[legID*3+1].Kp = 20;
        motorCmd[legID*3+1].Kd = 3;
    }
    void setStableGain(){
        for(int i(0); i<4; ++i){
            setStableGain(i);
        }
    }



    void setDynamicsGain(int legID){
        motorCmd[legID*3+0].mode = 1;
        motorCmd[legID*3+0].Kp = 44;
        motorCmd[legID*3+0].Kd = 2;
        motorCmd[legID*3+1].mode = 1;
        motorCmd[legID*3+1].Kp = 44;
        motorCmd[legID*3+1].Kd = 2;
    }
    void setDynamicsGain(){
        for(int i(0); i<4; ++i){
            setDynamicsGain(i);
        }
    }
    void setStableTrotGain(int legID){
        motorCmd[legID*3+0].mode = 1;
        motorCmd[legID*3+0].Kp = 20;   //10
        motorCmd[legID*3+0].Kd = 3;
        motorCmd[legID*3+1].mode = 1;
        motorCmd[legID*3+1].Kp = 20;   //10
        motorCmd[legID*3+1].Kd = 3;

    }
    void setStableTrotGain(){
        for(int i(0); i<4; ++i){
            setStableTrotGain(i);
        }
    }

    void setWheelStopGain(int legID){
        motorCmd[legID*3+2].mode = 1;
        motorCmd[legID*3+2].Kp =0;
        motorCmd[legID*3+2].Kd = 2;     //LF_wheel
        motorCmd[legID*3+2].mode = 1;
        motorCmd[legID*3+2].Kp = 0;
        motorCmd[legID*3+2].Kd = 2;
        motorCmd[legID*3+2].mode = 1;
        motorCmd[legID*3+2].Kp = 0;
        motorCmd[legID*3+2].Kd = 2;
        motorCmd[legID*3+2].mode = 1;
        motorCmd[legID*3+2].Kp = 0;
        motorCmd[legID*3+2].Kd = 2;
    }
    void setWheelStopGain(){
        for(int i(0); i<4; ++i){
            setWheelStopGain(i);
        }
    }

    void setWheelRollGain(int legID){
        motorCmd[legID*3+2].mode = 1;
        motorCmd[legID*3+2].Kp =0;
        motorCmd[legID*3+2].Kd = 0;     //LF_wheel
        motorCmd[legID*3+2].mode = 1;
        motorCmd[legID*3+2].Kp = 0;
        motorCmd[legID*3+2].Kd = 0;
        motorCmd[legID*3+2].mode = 1;
        motorCmd[legID*3+2].Kp = 0;
        motorCmd[legID*3+2].Kd = 0;
        motorCmd[legID*3+2].mode = 1;
        motorCmd[legID*3+2].Kp = 0;
        motorCmd[legID*3+2].Kd = 0;
    }
    void setWheelRollGain(){
        for(int i(0); i<4; ++i){
            setWheelRollGain(i);
        }
    }

    void setWheelConGain(int legID){
        motorCmd[legID*3+2].mode = 1;
        motorCmd[legID*3+2].Kp = 3;
        motorCmd[legID*3+2].Kd = 1.5;     //LF_wheel
        motorCmd[legID*3+2].mode = 1;
        motorCmd[legID*3+2].Kp = 3;
        motorCmd[legID*3+2].Kd = 1.5;
        motorCmd[legID*3+2].mode = 1;
        motorCmd[legID*3+2].Kp = 3;
        motorCmd[legID*3+2].Kd = 1.5;
        motorCmd[legID*3+2].mode = 1;
        motorCmd[legID*3+2].Kp = 3;
        motorCmd[legID*3+2].Kd = 1.5;
    }
    void setWheelConGain(){
        for(int i(0); i<4; ++i){
            setWheelConGain(i);
        }
    }
    void setSwingGain(int legID){
        // motorCmd[legID*3+0].mode = 1;
        // motorCmd[legID*3+0].Kp = 120;
        // motorCmd[legID*3+0].Kd = 1.2;
        // motorCmd[legID*3+1].mode = 1;
        // motorCmd[legID*3+1].Kp = 120;
        // motorCmd[legID*3+1].Kd = 1.2;
        motorCmd[legID*3+0].mode = 1;
        motorCmd[legID*3+0].Kp = 280;
        motorCmd[legID*3+0].Kd = 1.2;
        motorCmd[legID*3+1].mode = 1;
        motorCmd[legID*3+1].Kp = 280;
        motorCmd[legID*3+1].Kd = 1.2;
    }

    
    void setSwingGain(){
        for(int i(0); i<4; ++i){
            setSwingGain(i);
        }
    }
    void setChangeGain(int legID){
        motorCmd[legID*3+0].mode = 1;
        motorCmd[legID*3+0].Kp = 180;
        motorCmd[legID*3+0].Kd = 5;
        motorCmd[legID*3+1].mode = 1;
        motorCmd[legID*3+1].Kp = 180;
        motorCmd[legID*3+1].Kd = 5;
    }
    void setChangeGain(){
        for(int i(0); i<4; ++i){
            setChangeGain(i);
        }
    }


};
// template struct LowlevelCmd<double>;
template struct LowlevelCmd<float>;


#endif  //LOWLEVELCMD_H