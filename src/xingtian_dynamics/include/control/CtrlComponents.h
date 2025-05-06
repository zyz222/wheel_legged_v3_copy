/**********************************************************************
 Copyright (c) 2020-2023, ZYZ. All rights reserved.
***********************************************************************/
#ifndef CTRLCOMPONENTS_H
#define CTRLCOMPONENTS_H

#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "interface/IOInterface.h"
#include "interface/CmdPanel.h"
#include "common/xingtianrobot.h"
#include "Gait/WaveGenerator.h"      //文件需要详细修改！！
#include "control/Estimator.h"
#include "control/BalanceCtrl.h"
#include <string>
#include <iostream>
#include "Controllers/DesiredStateCommand.h"
#include "common/cppTypes.h"
#include "Gait/GaitScheduler.h"
#include "common/Quadruped.h"
#include "control/xingtian_parameters.h"
#include "common/xingtian.h"
#pragma once
template <typename T> class DesiredStateCommand;  // 前向声明
template <typename T> class GaitScheduler;
template <typename T> class WaveGenerator;
template <typename T> class Estimator;
// 后续代码...
template <typename T>
struct CtrlComponents{
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CtrlComponents(IOInterface<T> *ioInter):ioInter(ioInter)
    {
        lowCmd = new LowlevelCmd<T>();
        lowState = new LowlevelState<T>();
        contact = new Vec4<T>;
        phase = new Vec4<T>;
        *contact = Vec4<T>(0, 0, 0, 0);
        *phase = Vec4<T>(0.5, 0.5, 0.5, 0.5);
    }
    ~CtrlComponents(){
        delete lowCmd;
        delete lowState;
        delete ioInter;
        delete robotModel;
        delete waveGen;
        delete estimator;
        delete balCtrl;
        delete _desiredStateCommand;
        delete _gaitScheduler;
        delete contact;
        delete phase;
        delete _xingtianParameters;
        // delete _xingtian_model;
    }
    
    LowlevelCmd<T> *lowCmd;      //前两个相当于leg_controller  
    LowlevelState<T> *lowState;   //腿部状态估计的数据
    IOInterface<T> *ioInter;       //相当于userparameters
    QuadrupedRobot<T> *robotModel;   //机器人动力学模型
    Quadruped<T> _xingtian_model;   //模型参数

    GaitScheduler<T>* _gaitScheduler;        //步态规划器 这个是规划什么步态 包括相位偏移等
    WaveGenerator<T> *waveGen;    //里边是需要详细修改的！！ 波形生成器

    DesiredStateCommand<T> *_desiredStateCommand;     //期望的状态指令
    
    Estimator<T> *estimator;
    BalanceCtrl<T> *balCtrl;
    // VisualizationData<T> *visualizationData;                 //可视化数据
    
    Vec4<T> *contact;                        //在stateEstimatorcontainer 中也包含有
    Vec4<T> *phase;

   

    XingtianParameters<T>* _xingtianParameters;   //步态参数

    // KeyBoard<T>* _keyboard;

    double dt;
    bool *running;
    CtrlPlatform ctrlPlatform;


    void sendRecv(){
        ioInter->sendRecv(lowCmd, lowState);
    }

    void runWaveGen(){
        // std::cout << "runWaveGen,生成波形" << std::endl;
        waveGen->calcContactPhase(*phase, *contact, _waveStatus);
    }

    void setAllStance(){
        _waveStatus = WaveStatus::STANCE_ALL;
    }

    void setAllSwing(){
        _waveStatus = WaveStatus::SWING_ALL;
    }

    void setStartWave(){
        _waveStatus = WaveStatus::WAVE_ALL;
    }

    void geneObj(){
        estimator = new Estimator<T>(robotModel, lowState, contact, phase, dt);
        balCtrl = new BalanceCtrl<T>(robotModel);
        // _desiredStateCommand =new DesiredStateCommand<T>(ioInter->getKeyboard(),ioInter->getRCControl(),estimator,dt);
        _desiredStateCommand =new DesiredStateCommand<T>(ioInter->getKeyboard(),ioInter->getRCControl(),dt);
    
        //build model
        _xingtian_model = buildXingtian<T>();
        // Initialize the model and robot data
        // _model = _xingtian_model.buildModel();         //建立浮动基座模型，用于动力学计算
    }
    T gait_type;

private:
    WaveStatus _waveStatus = WaveStatus::SWING_ALL;

};
// template struct CtrlComponents<double>;
template struct CtrlComponents<float>;

#endif  // CTRLCOMPONENTS_H