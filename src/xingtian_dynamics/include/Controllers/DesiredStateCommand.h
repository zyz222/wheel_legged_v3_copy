/*!
 * @file DesiredStateCommand.h
 * @brief Logic to convert a joystick command into a desired trajectory for the robot
 *
 * This will generate a state trajectory which can easily be used for model predictive controllers
 */
//这里是生成期望的状态轨迹！！！必须有！！！
// 接收遥控器的指令生成轨迹状态信息


/*========================= Gamepad Control ==========================*/
/**
 *
 */
#ifndef DESIRED_STATE_COMMAND_H
#define DESIRED_STATE_COMMAND_H

#include <iostream>

#include "control/Estimator.h"
#include "common/cppTypes.h"
// #include "Controllers/StateEstimatorContainer.h"
#include "interface/KeyBoard.h"       //获取仿真用户指令
#include "interface/CmdPanel.h"
#include "real_robot/rt_rc_interface.h"    //获取真机遥控器指令

/**
 *
 */
template <typename T>
struct DesiredStateData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // DesiredStateData():stateDes(Vec12<T>::Zero()),pre_stateDes(Vec12<T>::Zero()) { zero(); }    //清零期望的状态数据
  DesiredStateData() { zero(); }
  // Zero out all of the data
  void zero();

  // Instantaneous desired state command
  Vec12<T> stateDes;           //当前状态指令 线速度，角速度，位置，和姿态 3*4

  Vec12<T> pre_stateDes;     //上一次状态指令

  // Desired future state trajectory (for up to 10 timestep MPC) 十倍步长的轨迹
  Eigen::Matrix<T, 12, 10> stateTrajDes;       //状态轨迹
};

/**
 *
 */
template <typename T>
class DesiredStateCommand {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Initialize with the GamepadCommand struct    KeyBoard里边是包含了指令和数值
  DesiredStateCommand(CmdPanel* command, rc_control_settings* rc_command,
                      Estimator<T>* sEstimate, T _dt) :cmdPanel(command), rcCommand(rc_command), stateEstimate(sEstimate) 
                      {
    // cmdPanel = command;
    // rcCommand = rc_command;
    // stateEstimate = sEstimate;
    data.stateDes.setZero();
    
    data.pre_stateDes.setZero();
    leftAnalogStick.setZero();
    rightAnalogStick.setZero();
    dt = _dt;
     
  }
  Vec2<T> leftAnalogStick;    //前进和航向角速度
  Vec2<T> rightAnalogStick;   //控制姿态，pitch roll
  T height;   //机身高度
  // Holds the instantaneous desired state and future desired state trajectory
  DesiredStateData<T> data; 
  // auto* data = new (Eigen::aligned_allocator<DesiredStateData<double>>()) DesiredStateData<double>();
// data->zero();
  void convertToStateCommands();
  //下边这几个函数都没有用到
  void setCommandLimits(T minVelX_in, T maxVelX_in,
                        T minVelY_in, T maxVelY_in, T minTurnRate_in, T maxTurnRate_in);
  void desiredStateTrajectory(int N, Vec10<T> dtVec);
  // void printRawInfo();
  void printStateCommandInfo();
 
  // These should come from the inferface
  T maxRoll = 0.4;
  T minRoll = -0.4;

  T maxPitch = 0.4;
  T minPitch = -0.4;

  T maxVelX = 3.0;
  T minVelX = -3.0;

  T maxVelY = 2.0;
  T minVelY = -2.0;

  T maxTurnRate = 2.5;
  T minTurnRate = -2.5;

  const rc_control_settings* rcCommand;
  const CmdPanel* cmdPanel;    // 获取用户指令

  // bool trigger_pressed = false;      //触发器 

private:
  Estimator<T>* stateEstimate;       //状态估计 
  // Dynamics matrix for discrete time approximation  离散时间近似的动力学矩阵
  Mat12<T> A;    
  // DesiredStateData<T> data;   
  // Control loop timestep change
  T dt;                                  //控制循环时间间隔

  // Value cutoff for the analog stick deadband
  T deadbandRegion = 0.075;      //遥控器的死区
  //const T filter = 0.01;
  const T filter = 0.1;

  // Choose how often to print info, every N iterations
  int printNum = 5;  // N*(0.001s) in simulation time

  // Track the number of iterations since last info print
  int printIter = 0;
};

#endif

