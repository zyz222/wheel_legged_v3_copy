#ifndef xingtian_CONVEXMPCLOCOMOTION_H
#define xingtian_CONVEXMPCLOCOMOTION_H

#include <Controllers/FootSwingTrajectory.h>
#include <control/CtrlComponents.h>
#include <SparseCMPC/SparseCMPC.h>
#include "common/cppTypes.h"
#include "convexMPC/Gait.h"
#include "convexMPC/RobotState.h"
#include <cstdio>

using Eigen::Array4f;     
using Eigen::Array4i;

  //这个就没用！！
template<typename T>
struct CMPC_Result {
  LegControllerCommand<float> commands[4];      // 4个足端控制指令
  Vec4<T> contactPhase;               // 4个足端接触状态
};                         



class ConvexMPCLocomotion {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ConvexMPCLocomotion(float _dt, int _iterations_between_mpc, CtrlComponents<float>* _ctrlComp);
  void initialize();

  template<typename T>
  void run(CtrlComponents<T>& data);   // 运行MPC
  // bool currently_jumping = false;      // 机器人是否正在跳跃
  // 机身坐标系下的
  Vec3<float> pBody_des;                // 机体系目标位置  
  Vec3<float> vBody_des;                // 机体系目标速度
  Vec3<float> aBody_des;                  // 机体系目标加速度

  Vec3<float> pBody_RPY_des;                // 机体系目标欧拉角
  Vec3<float> vBody_Ori_des;                // 机体系目标欧拉角速度

  Vec3<float> pFoot_des[4];                // 四足期望足端位置
  Vec3<float> vFoot_des[4];                // 四足期望足端速度
  Vec3<float> aFoot_des[4];                // 四足期望足端加速度

  Vec3<float> Fr_des[4];                          // 四足期望接触力

  Vec4<float> contact_state;                // 四足足端接触状态

private:
  void _SetupCommand(CtrlComponents<float> & data);     // 设置命令

  float _yaw_turn_rate;                     // yaw角速度
  float _yaw_des;                           // yaw角

  float _roll_des;                          // roll角
  float _pitch_des;                         // pitch角

  float _x_vel_des = 0.;                    // x方向速度
  float _y_vel_des = 0.;                    // y方向速度


  float _body_height = 0.25;


  void recompute_timing(int iterations_per_mpc);        //重新计算每次迭代的时间
  void updateMPCIfNeeded(int* mpcTable, CtrlComponents<float>& data, bool omniMode);      //根据迭代器步长，判断是否需要重新求解MPC
  void solveDenseMPC(int *mpcTable, CtrlComponents<float> &data);         //求解MPC  考虑全时域内的动力学约束
  void solveSparseMPC(int *mpcTable, CtrlComponents<float> &data);         //求解MPC  稀疏矩阵形式的MPC求解器，用于长时域问题
  void initSparseMPC();                        //初始化稀疏MPC求解器
  int iterationsBetweenMPC;                   //两次MPC求解时间之间的迭代次数
  int horizonLength;                         //MPC预测时域时间长度
  int default_iterations_between_mpc;                         //默认MPC迭代次数
  float dt;                      // 时间步长
  float dtMPC;                   // MPC时间步长
  int iterationCounter = 0;            // 迭代计数器
  Vec3<float> f_ff[4];
  Vec4<float> swingTimes;                                   // 摆动时间
  FootSwingTrajectory<float> footSwingTrajectories[4];       //足端摆动轨迹
  OffsetDurationGait trotting, standing, trotRunning, walking, walking2;    // 步态
  MixedFrequncyGait random, random2;                    //混合步态      
  Mat3<float> Kp, Kd, Kp_stance, Kd_stance;       
  bool firstRun = true;
  bool firstSwing[4];                                        // 摆动标志
  float swingTimeRemaining[4];                               // 剩余摆动时间
  float stand_traj[6];                                        // 站立轨迹
  int current_gait;                                           // 当前步态
  int gaitNumber;                                             // 步态数量

  Vec3<float> world_position_desired;                          // 世界坐标系下期望位置
  Vec3<float> rpy_int;                                         // 欧拉角
  Vec3<float> rpy_comp;                                         // 补偿
  float x_comp_integral = 0;                                    // x轴积分

  Vec3<float> pFoot[4];                                          // 四足足端位置 世界坐标系下
  CMPC_Result<float> result;                                   // 轨迹优化结果
  float trajAll[12*36];                                         // 

  CtrlComponents<float>* _CtrlComp = nullptr;
  float cmpc_x_drag = 3;

  vectorAligned<Vec12<double>> _sparseTrajectory;                  // 稀疏轨迹

  // SparseCMPC _sparseCMPC;

  int jcqp_max_iter = 10200;    //最大迭代次数
  double jcqp_rho = 1e-3;      //惩罚参数1    增大加速残差收敛，可能对偶残差振荡
  double jcqp_sigma = 1e-3;    //惩罚参数2    增大加速偶残差收敛，降低收敛速度
  double jcqp_alpha = 1.0;   //步长或者松弛因子，增大加快收敛，有可能发散， 减小 收敛稳定，但速度慢。
  double jcqp_terminate = 1e-4;    //终止条件，小于这个值则终止 增大，精度降低
  double use_jcqp = 0;       //是否使用jcqp，1使用，0不使用   0是简单模式，1是启用JCQP 2是启用高级JCQP

  int use_wbc = 0;    //0 不使用wbc，1使用wbc

};


#endif //xingtian_CONVEXMPCLOCOMOTION_H
