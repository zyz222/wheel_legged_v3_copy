/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef GAITGENERATOR_H
#define GAITGENERATOR_H

#include "Gait/WaveGenerator.h"
#include "Gait/FeetEndCal.h"


// 步态生成器
/*cycloid gait*/
template <typename T>
class GaitGenerator{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    GaitGenerator(CtrlComponents<T> *ctrlComp);
    ~GaitGenerator();
    void setGait(Vec2<T> vxyGoalGlobal, T dYawGoal, T gaitHeight);
    void run(Vec34<T> &feetPos, Vec34<T> &feetVel);
    Vec3<T> getFootPos(int i);
    Vec3<T> getFootVel(int i);
    void restart();
private:
    T cycloidXYPosition(T startXY, T endXY, T phase);
    T cycloidXYVelocity(T startXY, T endXY, T phase);
    T cycloidZPosition(T startZ, T height, T phase);
    T cycloidZVelocity(T height, T phase);

    WaveGenerator<T> *_waveG;
    Estimator<T> *_est;
    FeetEndCal<T> *_feetCal;
    QuadrupedRobot<T> *_robModel;

    LowlevelState<T> *_lowState;

    T _gaitHeight;
    Vec2<T> _vxyGoal;
    T _dYawGoal;
    Vec4<T> *_phase, _phasePast;
    Vec4<T> *_contact;
    Vec34<T> _startP, _endP, _idealP, _pastP;
    bool _firstRun;


};

#endif  // GAITGENERATOR_H