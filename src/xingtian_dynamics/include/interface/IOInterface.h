/**********************************************************************
 Copyright (c) 2024, CIEM-ZYZ. All rights reserved.
***********************************************************************/
#ifndef IOINTERFACE_H
#define IOINTERFACE_H
#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "interface/CmdPanel.h"
#include <string>
#include "interface/KeyBoard.h"
#include "real_robot/rt_rc_interface.h"
template <typename T>
class IOInterface
{
public:
    IOInterface(){}
    ~IOInterface(){delete _keyboard;}
    virtual void sendRecv(const LowlevelCmd<T> *cmd, LowlevelState<T> *state) = 0;

    void zeroCmdPanel(){_keyboard->setZero();}
    void setPassive(){_keyboard->setPassive();}
    CmdPanel* getKeyboard() const { return _keyboard; }
    // virtual KeyBoard<T>* getKeyboard() = 0;
    virtual rc_control_settings* getRCControl() = 0;
protected:
    CmdPanel *_keyboard;    //在子类中分配内存，在父类中使用
};

#endif  //IOINTERFACE_H