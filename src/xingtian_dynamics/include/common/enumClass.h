/**********************************************************************
 Copyright (c) CIEM-ZYZ-2024. All rights reserved.
***********************************************************************/
#ifndef ENUMCLASS_H
#define ENUMCLASS_H

#include <iostream>
#include <sstream>

enum class CtrlPlatform{
    GAZEBO,
    REALROBOT,
};

enum class RobotType{
    A1,
    Go1,
    xingtian
};

enum class UserCommand{
    // EXIT,
    NONE,
    START,      // trotting      
    L2_A,       // fixedStand
    L2_B,       // passive
    L2_X,       // freeStand
    ROLL,        //wheel

    L1_X,       // balanceTest
    L1_A,       // swingTest
    L1_Y        // stepTest
};

enum class FrameType{
    BODY,
    HIP,
    GLOBAL
};

enum class WaveStatus{
    STANCE_ALL,
    SWING_ALL,
    WAVE_ALL
};

enum class FSMMode{
    NORMAL,
    CHANGE
};

enum class FSMStateName{
    // EXIT,
    INVALID,
    PASSIVE,
    FIXEDSTAND,
    FREESTAND,
    TROTTING,
    BALANCETEST,
    SWINGTEST,
    STEPTEST,
    WHEELTEST
};

#endif  // ENUMCLASS_H