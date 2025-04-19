/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef KEYBOARD_H
#define KEYBOARD_H

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include "interface/CmdPanel.h"
#include "common/mathTools.h"
#include "common/cppTypes.h"
template <typename T>
class KeyBoard : public CmdPanel{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    KeyBoard();
    Vec2<double> getLeftStickAnalog() const override { return leftStickAnalog; }
    // Vec2<T> getLeftStickAnalog() const { }
    // 返回副本
    // Vec2<T> getRightStickAnalog() const { }
    Vec2<double> getRightStickAnalog() const override { return rightStickAnalog; }
    double getHeight() const override { return height; }
    // T getHeight() const {}
    ~KeyBoard();
private:
    static void* runKeyBoard(void *arg);
    void* run(void *arg);
    UserCommand checkCmd();
    void changeValue();
    Vec2<T> leftStickAnalog;     //控制XY 航向角速度.X方向和yaw的角速度
    Vec2<T> rightStickAnalog;    //控制机身姿态，roll和pitch角速度
    T height;   //控制机身高度

    pthread_t _tid;
    T sensitivityLeft = 0.05;
    T sensitivityRight = 0.05;
    struct termios _oldSettings, _newSettings;
    fd_set set;
    int res;
    int ret;
    char _c;
};

#endif  // KEYBOARD_H
