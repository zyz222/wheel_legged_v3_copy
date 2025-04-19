/**********************************************************************
 Copyright (c) CIEM-ZYZ-2024. All rights reserved.
***********************************************************************/
#ifndef XINGTIANLEG_H
#define XINGTIANLEG_H

#include "common/cppTypes.h"
#include "common/enumClass.h"
#include "message/LowlevelState.h"

#include "message/LowlevelCmd.h"

template <typename T>
class QuadrupedLeg{
public:
    QuadrupedLeg(int legID, T hipLinkLength, 
                 T kneeLinkLength, Vec3<T> pHip2B);
    ~QuadrupedLeg(){}
    Vec3<T> calcPEe2H(Vec3<T> q);   //足端到髋关节坐标
    Vec3<T> calcPEe2B(Vec3<T> q);   //足端到机体中心
    Vec3<T> calcVEe(Vec3<T> q, Vec3<T> qd);
    Vec3<T> calcQ(Vec3<T> pEe, FrameType frame);

    Vec3<T> calcQd(Vec3<T> q, Vec3<T> vEe,Vec3<T> qd);
    Vec3<T> calcQd(Vec3<T> pEe, Vec3<T> vEe, Vec3<T> qd,FrameType frame);
    Vec3<T> calcTau(Vec3<T> q, Vec3<T> force);
    Mat3<T> calcJaco(Vec3<T> q);
    Vec3<T> getHip2B(){return _pHip2B;}
protected:
    LowlevelState<T> *_lowState;
    T q2_ik(T gama);
    T q1_ik(T q1, T px,  T pz,T hip_l1, T knee_l2);
    T judge_result(T px, T pz);
    T _sideSign;
    const T _hipLinkLength, _kneeLinkLength;
    const Vec3<T> _pHip2B;   //平移矩阵 0.2005
        
    Vec3<T> lastQResult;  // 新增成员变量保存上一时刻关节角度
    bool hasLastResult = false;  // 标记是否有历史数据

};

template <typename T>
class xingtianLeg : public QuadrupedLeg<T>{
public:
    xingtianLeg(const int legID, const Vec3<T> pHip2B):
        QuadrupedLeg<T>(legID, 0.140, 0.140, pHip2B){}   //m
    ~xingtianLeg(){}
};

#endif  // XINGTIANLEG_H