#pragma once
#ifndef _convexmpc_interface
#define _convexmpc_interface
#define K_MAX_GAIT_SEGMENTS 36

//#include "common_types.h"

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

struct problem_setup
{
  float dt;             //MPC的步长
  float mu;            // friction coefficient   摩擦力
  float f_max;       //输出的最大力
  int horizon;         //MPC的预测步长
};




struct update_data_t                //更新数据
{ 
  float p[3];                       //位置
  float v[3];                        //速度
  float q[4];                        //四元数
  float w[3];                        //角速度
  float r[12];                        //足端力
  float yaw;                          //偏航角
  float weights[12];                    //权重
  float traj[12*K_MAX_GAIT_SEGMENTS];      //步态轨迹
  float alpha;                            //优化算法的步长或平滑系数，调节轨迹跟踪的响应速度
  unsigned char gait[K_MAX_GAIT_SEGMENTS];        //步态的模式数组，
  
  int max_iterations;                            //最大迭代次数
  double rho, sigma, solver_alpha, terminate;     //惩罚因子，正则话系数，求解器的步长，收敛阈值
  int use_jcqp;                                   //是否使用jcqp求解器
  float x_drag;                        //x方向的拖拽系数
};

EXTERNC void setup_problem(double dt, int horizon, double mu, double f_max);
EXTERNC void update_problem_data(double* p, double* v, double* q, double* w, double* r, double yaw, double* weights, double* state_trajectory, double alpha, int* gait);
EXTERNC double get_solution(int index);
EXTERNC void update_solver_settings(int max_iter, double rho, double sigma, double solver_alpha, double terminate, double use_jcqp);
EXTERNC void update_problem_data_floats(float* p, float* v, float* q, float* w,
                                        float* r, float yaw, float* weights,
                                        float* state_trajectory, float alpha, int* gait);

void update_x_drag(float x_drag);
#endif