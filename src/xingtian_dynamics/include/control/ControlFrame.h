/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef CONTROLFRAME_H
#define CONTROLFRAME_H

#include "FSM/FSM.h"
#include "control/CtrlComponents.h"
template <typename T>
class ControlFrame{
public:
	ControlFrame(CtrlComponents<T> *ctrlComp);
	~ControlFrame(){
		delete _FSMController;
	}
	void run();
private:
	FSM<T>* _FSMController;
	CtrlComponents<T> *_ctrlComp;
};

#endif  //CONTROLFRAME_H