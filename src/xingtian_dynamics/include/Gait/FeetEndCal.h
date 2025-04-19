/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef FEETENDCAL_H
#define FEETENDCAL_H

#include "control/CtrlComponents.h"
#include "message/LowlevelState.h"
template <typename T>
class FeetEndCal{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FeetEndCal(CtrlComponents<T> *ctrlComp);
    ~FeetEndCal();
    Vec3<T> calFootPos(int legID, Vec2<T> vxyGoalGlobal, T dYawGoal, T phase);
private:
    LowlevelState<T> *_lowState;
    Estimator<T> *_est;
    QuadrupedRobot<T> *_robModel;

    Vec3<T> _nextStep, _footPos;
    Vec3<T> _bodyVelGlobal;        // linear velocity
    Vec3<T> _bodyAccGlobal;        // linear accelerator
    Vec3<T> _bodyWGlobal;          // angular velocity

    Vec4<T> _feetRadius, _feetInitAngle;
    T _yaw, _dYaw, _nextYaw;

    T _Tstance, _Tswing;
    T _kx, _ky, _kyaw;
};

#endif  // FEETENDCAL_H