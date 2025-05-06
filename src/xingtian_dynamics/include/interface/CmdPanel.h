/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#ifndef A41FA33F_E17E_4530_A9A7_CAC10CB03BF0
#define A41FA33F_E17E_4530_A9A7_CAC10CB03BF0
#ifndef CMDPANEL_H
#define CMDPANEL_H

#include "message/unitree_joystick.h"
#include "common/enumClass.h"
#include <pthread.h>
#include "common/cppTypes.h"

struct UserValue{
    float lx;   //roll
    float ly;   //pitch
    float rx;   //yaw
    float ry;   //height
    float L2;   
    float vx;    
    float yaw_rate;
    
    UserValue(){
        setZero();
    }
    void setZero(){
        lx = 0;
        ly = 0;
        rx = 0;
        ry = 0;
        L2 = 0;    //期望高度
        vx = 0;
        yaw_rate= 0;
    }
};
// template <typename T>
class CmdPanel{
public:
    CmdPanel(){}
    virtual ~CmdPanel(){}
    UserCommand getUserCmd()const {return userCmd;}
    UserValue getUserValue()const {return userValue;}

    virtual Vec2<float> getLeftStickAnalog() const = 0;
    virtual Vec2<float> getRightStickAnalog() const = 0;
    virtual float getHeight() const = 0;
    

    void setPassive(){userCmd = UserCommand::L2_B;}
    void setZero(){userValue.setZero();}

protected:
    virtual void* run(void *arg){return NULL;}
    UserCommand userCmd;
    UserValue userValue;
};

#endif  // CMDPANEL_H


#endif /* A41FA33F_E17E_4530_A9A7_CAC10CB03BF0 */
