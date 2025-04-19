/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef FSMSTATE_H
#define FSMSTATE_H

#include <string>
#include <iostream>
#include <unistd.h>
#include "control/CtrlComponents.h"
#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "common/enumClass.h"
#include "common/mathTools.h"
#include "common/cppTypes.h"
#include "common/timeMarker.h"
#include "interface/CmdPanel.h"
template <typename T>
class FSMState{
public:
    FSMState(CtrlComponents<T> *ctrlComp, FSMStateName stateName, std::string stateNameString);

    virtual void enter() = 0;
    virtual void run() = 0;
    virtual void exit() = 0;
    virtual FSMStateName checkChange() {return FSMStateName::INVALID;}

    FSMStateName _stateName;
    std::string _stateNameString;        //当前状态

    void jointPDControl(int leg, Vec3<T> qDes, Vec3<T> qdDes);   //关节位置控制器
    void cartesianImpedanceControl(int leg, Vec3<T> pDes, Vec3<T> vDes,       //笛卡尔阻抗控制
        Vec3<double> kp_cartesian,
        Vec3<double> kd_cartesian);
    void footstepHeuristicPlacement(int leg);    //足端位置启发式定位  不用实现

    void runControls();
    void runBalanceController();
        //下边三个是没有具体实现的
    void runWholeBodyController();
    void runConvexModelPredictiveController();
    void runRegularizedPredictiveController();   //正规化的预测控制

    void turnOnAllSafetyChecks();
    void turnOffAllSafetyChecks();

    // Pre controls safety checks
    bool checkSafeOrientation = false;  // check roll and pitch

    // Post control safety checks
    bool checkPDesFoot = false;          // do not command footsetps too far
    bool checkForceFeedForward = false;  // do not command huge forces
    bool checkLegSingularity = false;    // do not let leg

    // Leg controller command placeholders for the whole robot (3x4 matrices)
    Mat34<T> jointFeedForwardTorques;  // feed forward joint torques
    Mat34<T> jointPositions;           // joint angle positions
    Mat34<T> jointVelocities;          // joint angular velocities
    Mat34<T> footFeedForwardForces;    // feedforward forces at the feet
    Mat34<T> footPositions;            // cartesian foot positions
    Mat34<T> footVelocities;           // cartesian foot velocities
    // Footstep locations for next step
    Mat34<T> footstepLocations;

    BalanceCtrl<T> *_balCtrl;       // balance控制器
protected:
    CtrlComponents<T> *_ctrlComp;
    FSMStateName _nextStateName;

    LowlevelCmd<T> *_lowCmd;
    LowlevelState<T> *_lowState;
    UserValue _userValue;
    
private:
    // Create the cartesian P gain matrix
    Mat3<float> kpMat; 

    // Create the cartesian D gain matrix
    Mat3<float> kdMat;

};

#endif  // FSMSTATE_H