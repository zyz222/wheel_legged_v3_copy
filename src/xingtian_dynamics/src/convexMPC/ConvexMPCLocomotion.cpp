#include <iostream>
#include <Utilities/Timer.h>
#include <Utilities/Utilities_print.h>

#include "convexMPC/ConvexMPCLocomotion.h"
#include "convexMPC/convexMPC_interface.h"


#include "convexMPC/Gait.h"


////////////////////
// Controller
////////////////////

ConvexMPCLocomotion::ConvexMPCLocomotion(float _dt, int _iterations_between_mpc, CtrlComponents<float>* _ctrlComp) :
  iterationsBetweenMPC(_iterations_between_mpc),
  horizonLength(10),    //预测10步
  dt(_dt),
  trotting(horizonLength, Vec4<int>(0,5,5,0), Vec4<int>(5,5,5,5),"Trotting"),
  standing(horizonLength, Vec4<int>(0,0,0,0),Vec4<int>(10,10,10,10),"Standing"),

  trotRunning(horizonLength, Vec4<int>(0,5,5,0),Vec4<int>(4,4,4,4),"Trot Running"),

  walking(horizonLength, Vec4<int>(0,3,5,8), Vec4<int>(5,5,5,5), "Walking"),
  walking2(horizonLength, Vec4<int>(0,5,5,0), Vec4<int>(7,7,7,7), "Walking2"),
  random(horizonLength, Vec4<int>(9,13,13,9), 0.4, "Flying nine thirteenths trot"),
  random2(horizonLength, Vec4<int>(8,16,16,8), 0.5, "Double Trot")
{
  _CtrlComp = _ctrlComp;
  dtMPC = dt * iterationsBetweenMPC;
  default_iterations_between_mpc = iterationsBetweenMPC;
  printf("[Convex MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n", dt, iterationsBetweenMPC, dtMPC);
  setup_problem(dtMPC, horizonLength, 0.4, 300);     //摩擦系数，最大支撑力
  
  rpy_comp[0] = 0;  
  rpy_comp[1] = 0;
  rpy_comp[2] = 0;
  rpy_int[0] = 0;
  rpy_int[1] = 0;
  rpy_int[2] = 0;

  for(int i = 0; i < 4; i++)
    firstSwing[i] = true;

  // initSparseMPC();    //初始化稀疏MPC

   pBody_des.setZero();        //质点位置  机身坐标系下期望的位置
   vBody_des.setZero();       //质点速度
   aBody_des.setZero();         //质点加速度
}

void ConvexMPCLocomotion::initialize(){
  for(int i = 0; i < 4; i++) firstSwing[i] = true;
  firstRun = true;
}

void ConvexMPCLocomotion::recompute_timing(int iterations_per_mpc) {
  iterationsBetweenMPC = iterations_per_mpc;
  dtMPC = dt * iterations_per_mpc;
}

void ConvexMPCLocomotion::_SetupCommand(CtrlComponents<float> & data){
  if(data._xingtian_model._robotType == RobotType::xingtian){
    _body_height = 0.29;
  }else{
    assert(false);
  }

  float x_vel_cmd, y_vel_cmd;
  float filter(0.1);
  _yaw_turn_rate = data._desiredStateCommand->rightAnalogStick[0];

  x_vel_cmd = data._desiredStateCommand->leftAnalogStick[1];
  y_vel_cmd = data._desiredStateCommand->leftAnalogStick[0];

  _x_vel_des = _x_vel_des*(1-filter) + x_vel_cmd*filter;
  _y_vel_des = _y_vel_des*(1-filter) + y_vel_cmd*filter;

  _yaw_des = data.estimator->stateEstimate.rpy[2] + dt * _yaw_turn_rate;
  _roll_des = 0.;
  _pitch_des = 0.;

}

template<>
void ConvexMPCLocomotion::run(CtrlComponents<float>& data) {
  bool omniMode = false;     //全向模式，直接使用机身坐标系下的速度，忽略机身旋转

  // Command Setup
  _SetupCommand(data);
 
  if(gaitNumber >= 10) {
    gaitNumber -= 10;
    omniMode = true;
  }

  auto& seResult = data.estimator->stateEstimate;

  // Check if transition to standing
  if(((gaitNumber == 4) && current_gait != 4) || firstRun)       //站立
  {
    stand_traj[0] = seResult.position[0];            
    stand_traj[1] = seResult.position[1];
    stand_traj[2] = seResult.position[2];

    stand_traj[3] = 0;
    stand_traj[4] = 0;
    stand_traj[5] = seResult.rpy[2];                  //yaw
    world_position_desired[0] = stand_traj[0];       //世界坐标系下的期望位置
    world_position_desired[1] = stand_traj[1];  
  }

  // pick gait
  Gait* gait = &trotting;     //默认就是trotting
  if(gaitNumber == 3)
    gait = &random;
  else if(gaitNumber == 4)
    gait = &standing;
  else if(gaitNumber == 5)
    gait = &trotRunning;

  else if(gaitNumber == 7)
    gait = &random2;
  current_gait = gaitNumber;

// 这块没看懂
  gait->setIterations(iterationsBetweenMPC, iterationCounter);
 


  // integrate position setpoint
  Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);           //x y 方向的期望速度，机身坐标系下的
  Vec3<float> v_des_world =  
    omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
  Vec3<float> v_robot = seResult.vWorld;

  //pretty_print(v_des_world, std::cout, "v des world");

  //Integral-esque pitche and roll compensation
  if(fabs(v_robot[0]) > .2)   //avoid dividing by zero
  {
    rpy_int[1] += dt*(_pitch_des - seResult.rpy[1])/v_robot[0];
  }
  if(fabs(v_robot[1]) > 0.1)
  {
    rpy_int[0] += dt*(_roll_des - seResult.rpy[0])/v_robot[1];
  }

  rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);
  rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);
  rpy_comp[1] = v_robot[0] * rpy_int[1];
  rpy_comp[0] = v_robot[1] * rpy_int[0] * (gaitNumber!=8);  //turn off for pronking


  for(int i = 0; i < 4; i++) {
    pFoot[i] = data.estimator->cheater_getFootPos(i);      //获取世界坐标系下的足端位置   ，
   
  }

  if(gait != &standing) {
    world_position_desired += dt * Vec3<float>(v_des_world[0], v_des_world[1], 0);
  }

  // some first time initialization
  if(firstRun)
  {
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.rpy[2];

    for(int i = 0; i < 4; i++)
    {
                                                                   //世界坐标系下规划
      footSwingTrajectories[i].setHeight(0.05);
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);       //获取四条腿的轨迹
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);        //设置世界坐标系下的最终位置

    }
    firstRun = false;
  }

  // foot placement
  for(int l = 0; l < 4; l++)
    swingTimes[l] = gait->getCurrentSwingTime(dtMPC, l);             //获取当前腿的摆动时间

  float side_sign[4] = {1, 1, -1, -1};
  // float interleave_y[4] = {-0.08, 0.08, 0.02, -0.02};
  float interleave_y[4] = {0, 0, 0, 0};
  //float interleave_gain = -0.13;
  float interleave_gain = 0;     // 摆动腿交错补偿
  //float v_abs = std::fabs(seResult.vBody[0]);
  float v_abs = std::fabs(v_des_robot[0]);                           //获取当前机器人的x方向速度 机身坐标系下的
  for(int i = 0; i < 4; i++)
  {

    if(firstSwing[i]) {
      swingTimeRemaining[i] = swingTimes[i];
    } else {
      swingTimeRemaining[i] -= dt;
    }
    //if(firstSwing[i]) {
    //footSwingTrajectories[i].setHeight(.05);
    footSwingTrajectories[i].setHeight(.06);
    Vec3<float> offset(0, side_sign[i] * 0, 0);

    Vec3<float> pRobotFrame = (data.robotModel->getFootPosition(*data.lowState,i,FrameType::HIP)).template cast<float>();    //解指针引用

    // pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain;
    float stance_time = gait->getCurrentStanceTime(dtMPC, i);     //站立相位的时间
    // 这里肯定是有问题的！！！
    
    Vec3<float> pYawCorrected =                                //修正足端位置
      coordinateRotation(CoordinateAxis::Z, -_yaw_turn_rate* stance_time / 2) * pRobotFrame;


    Vec3<float> des_vel;
    des_vel[0] = _x_vel_des;
    des_vel[1] = _y_vel_des;
    des_vel[2] = 0.0;

    Vec3<float> Pf = seResult.position + seResult.rBody.transpose() * (pYawCorrected
          + des_vel * swingTimeRemaining[i]);

    //+ seResult.vWorld * swingTimeRemaining[i];

    //float p_rel_max = 0.35f;
    float p_rel_max = 0.3f;    //每次最大走0.3米

    // Using the estimated velocity is correct
    //Vec3<float> des_vel_world = seResult.rBody.transpose() * des_vel;
    float pfx_rel = seResult.vWorld[0] * (.5 ) * stance_time +          //速度前馈项    
      .03f*(seResult.vWorld[0]-v_des_world[0]) +                                   //速度误差反馈项
      (0.5f*seResult.position[2]/9.81f) * (seResult.vWorld[1]*_yaw_turn_rate);      //离心力补偿项

    float pfy_rel = seResult.vWorld[1] * .5 * stance_time * dtMPC +                     //速度前馈项
      .03f*(seResult.vWorld[1]-v_des_world[1]) +                                       //速度误差反馈项
      (0.5f*seResult.position[2]/9.81f) * (-seResult.vWorld[0]*_yaw_turn_rate);               //离心力补偿项
    pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);      //补偿量限幅
    pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
    Pf[0] +=  pfx_rel;
    Pf[1] +=  pfy_rel;
    Pf[2] = -0.003;
    //Pf[2] = 0.0;
    footSwingTrajectories[i].setFinalPosition(Pf);             //设置摆动轨迹    设置最终的落脚点   世界坐标系下的位置

  }

  // calc gait
  iterationCounter++;

  // load LCM leg swing gains
  Kp << 700, 0, 0,
     0, 700, 0,
     0, 0, 150;
  Kp_stance = 0.05*Kp;


  Kd << 7, 0, 0,
     0, 7, 0,
     0, 0, 7;
  Kd_stance = Kd;
  // gait
  Vec4<float> contactStates = gait->getContactState();    //获取接触相位
  Vec4<float> swingStates = gait->getSwingState();
  int* mpcTable = gait->getMpcTable();                //获取MPC的步态表
  updateMPCIfNeeded(mpcTable, data, omniMode);

  //  StateEstimator* se = hw_i->state_estimator;
  // Vec4<float> se_contactState(0,0,0,0);



  for(int foot = 0; foot < 4; foot++)         //逐个腿进行控制
  {
    float contactState = contactStates[foot];                // 0: no contact, 1: full contact
    float swingState = swingStates[foot];
    if(swingState > 0) // foot is in swing
    {
      if(firstSwing[foot])
      {
        firstSwing[foot] = false;  
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);      //设置世界坐标系下的初始位置
      }

      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);      //计算贝塞尔曲线


      //      footSwingTrajectories[foot]->updateFF(hw_i->leg_controller->leg_datas[foot].q,
      //                                          hw_i->leg_controller->leg_datas[foot].qd, 0); // velocity dependent friction compensation todo removed
      //hw_i->leg_controller->leg_datas[foot].qd, fsm->main_control_settings.variable[2]);

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();            //世界坐标系下的期望位置
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();            //世界坐标系下的期望速度
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position);       //机身坐标系下的期望位置      
       
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);         //机身坐标系下的期望速度

      // Update for WBC
      pFoot_des[foot] = pDesFootWorld;
      vFoot_des[foot] = vDesFootWorld;
      aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration();
      
      if(!use_wbc){   //0不使用，1使用
        // Update leg control command regardless of the usage of WBIC
        data.lowCmd->commands[foot].pDes = pDesLeg;
        data.lowCmd->commands[foot].vDes = vDesLeg;
        data.lowCmd->commands[foot].kpCartesian = Kp;
        data.lowCmd->commands[foot].kdCartesian = Kd;
      }
    }
    else // foot is in stance
    {
      firstSwing[foot] = true;      //摆动腿结束了


      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      //cout << "Foot " << foot << " relative velocity desired: " << vDesLeg.transpose() << "\n";

      if(!use_wbc){
        data.lowCmd->commands[foot].pDes = pDesLeg;    
        data.lowCmd->commands[foot].vDes = vDesLeg;
        data.lowCmd->commands[foot].kpCartesian = Kp_stance;
        data.lowCmd->commands[foot].kdCartesian = Kd_stance;

        data.lowCmd->commands[foot].forceFeedForward = f_ff[foot];
        data.lowCmd->commands[foot].kdJoint = Mat3<float>::Identity() * 0.2;

        //      footSwingTrajectories[foot]->updateFF(hw_i->leg_controller->leg_datas[foot].q,
        //                                          hw_i->leg_controller->leg_datas[foot].qd, 0); todo removed
        // hw_i->leg_controller->leg_commands[foot].tau_ff += 0*footSwingController[foot]->getTauFF();
      }else{ // Stance foot damping
        data.lowCmd->commands[foot].pDes = pDesLeg;
        data.lowCmd->commands[foot].vDes = vDesLeg;
        data.lowCmd->commands[foot].kpCartesian = 0.*Kp_stance;
        data.lowCmd->commands[foot].kdCartesian = Kd_stance;
      }
      //            cout << "Foot " << foot << " force: " << f_ff[foot].transpose() << "\n";
      // se_contactState[foot] = contactState;

      // Update for WBC
      //Fr_des[foot] = -f_ff[foot];
    }
  }

  // se->set_contact_state(se_contactState); todo removed
  // data._stateEstimator->setContactPhase(se_contactState);

  // Update For WBC
  pBody_des[0] = world_position_desired[0];
  pBody_des[1] = world_position_desired[1];
  pBody_des[2] = _body_height;

  vBody_des[0] = v_des_world[0];
  vBody_des[1] = v_des_world[1];
  vBody_des[2] = 0.;

  aBody_des.setZero();

  pBody_RPY_des[0] = 0.;
  pBody_RPY_des[1] = 0.; 
  pBody_RPY_des[2] = _yaw_des;

  vBody_Ori_des[0] = 0.;
  vBody_Ori_des[1] = 0.;
  vBody_Ori_des[2] = _yaw_turn_rate;

  //contact_state = gait->getContactState();
  contact_state = gait->getContactState();       //
  // END of WBC Update


}

// 间隔触发优化
void ConvexMPCLocomotion::updateMPCIfNeeded(int *mpcTable, CtrlComponents<float> &data, bool omniMode) {
  //iterationsBetweenMPC = 30;       
  if((iterationCounter % iterationsBetweenMPC) == 0)
  {
    auto seResult = data.estimator->stateEstimate;      
    float* p = seResult.position.data();

    Vec3<float> v_des_robot(_x_vel_des, _y_vel_des,0);
    Vec3<float> v_des_world = omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
    //float trajInitial[12] = {0,0,0, 0,0,.25, 0,0,0,0,0,0};


    //printf("Position error: %.3f, integral %.3f\n", pxy_err[0], x_comp_integral);

    if(current_gait == 4)
    {
      float trajInitial[12] = {
        _roll_des,
        _pitch_des /*-hw_i->state_estimator->se_ground_pitch*/,
        (float)stand_traj[5]/*+(float)stateCommand->data.stateDes[11]*/,
        (float)stand_traj[0]/*+(float)fsm->main_control_settings.p_des[0]*/,
        (float)stand_traj[1]/*+(float)fsm->main_control_settings.p_des[1]*/,
        (float)_body_height/*fsm->main_control_settings.p_des[2]*/,
        0,0,0,0,0,0};

      for(int i = 0; i < horizonLength; i++)
        for(int j = 0; j < 12; j++)
          trajAll[12*i+j] = trajInitial[j];
    }

    else
    {
      const float max_pos_error = .1;
      float xStart = world_position_desired[0];
      float yStart = world_position_desired[1];

      if(xStart - p[0] > max_pos_error) xStart = p[0] + max_pos_error;
      if(p[0] - xStart > max_pos_error) xStart = p[0] - max_pos_error;

      if(yStart - p[1] > max_pos_error) yStart = p[1] + max_pos_error;
      if(p[1] - yStart > max_pos_error) yStart = p[1] - max_pos_error;

      world_position_desired[0] = xStart;
      world_position_desired[1] = yStart;

      float trajInitial[12] = {(float)rpy_comp[0],  // 0
        (float)rpy_comp[1],    // 1
        _yaw_des,    // 2
        //yawStart,    // 2
        xStart,                                   // 3
        yStart,                                   // 4
        (float)_body_height,      // 5
        0,                                        // 6
        0,                                        // 7
        _yaw_turn_rate,  // 8
        v_des_world[0],                           // 9
        v_des_world[1],                           // 10
        0};                                       // 11

      for(int i = 0; i < horizonLength; i++)
      {
        for(int j = 0; j < 12; j++)
          trajAll[12*i+j] = trajInitial[j];

        if(i == 0) // start at current position  TODO consider not doing this
        {
          //trajAll[3] = hw_i->state_estimator->se_pBody[0];
          //trajAll[4] = hw_i->state_estimator->se_pBody[1];
          trajAll[2] = seResult.rpy[2];
        }
        else
        {
          trajAll[12*i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC * v_des_world[0];
          trajAll[12*i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC * v_des_world[1];
          trajAll[12*i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * _yaw_turn_rate;
        }
      }
    }
    // Timer solveTimer;

    // if(_CtrlComp->cmpc_use_sparse > 0.5) {
      // solveSparseMPC(mpcTable, data);
    // } else {

      solveDenseMPC(mpcTable, data);
    // }
    //printf("TOTAL SOLVE TIME: %.3f\n", solveTimer.getMs());
  }

}

void ConvexMPCLocomotion::solveDenseMPC(int *mpcTable, CtrlComponents<float> &data) {
  auto seResult = data.estimator->stateEstimate;

  //float Q[12] = {0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2};

  float Q[12] = {0.25, 0.25, 10, 2, 2, 50, 0, 0, 0.3, 0.2, 0.2, 0.1};

  //float Q[12] = {0.25, 0.25, 10, 2, 2, 40, 0, 0, 0.3, 0.2, 0.2, 0.2};
  float yaw = seResult.rpy[2];
  float* weights = Q;
  float alpha = 4e-5; // make setting eventually
  //float alpha = 4e-7; // make setting eventually: DH
  float* p = seResult.position.data();
  float* v = seResult.vWorld.data();
  float* w = seResult.omegaWorld.data();
  float* q = seResult.orientation.data();

  float r[12];
  for(int i = 0; i < 12; i++)
    r[i] = pFoot[i%4][i/4]  - seResult.position[i/4];

  //printf("current posistion: %3.f %.3f %.3f\n", p[0], p[1], p[2]);

  if(alpha > 1e-4) {
    std::cout << "Alpha was set too high (" << alpha << ") adjust to 1e-5\n";
    alpha = 1e-5;
  }

  Vec3<float> pxy_act(p[0], p[1], 0);
  Vec3<float> pxy_des(world_position_desired[0], world_position_desired[1], 0);
  //Vec3<float> pxy_err = pxy_act - pxy_des;
  float pz_err = p[2] - _body_height;
  Vec3<float> vxy(seResult.vWorld[0], seResult.vWorld[1], 0);

  Timer t1;
  dtMPC = dt * iterationsBetweenMPC;
  setup_problem(dtMPC,horizonLength,0.4,300);
  //setup_problem(dtMPC,horizonLength,0.4,650); //DH
  update_x_drag(x_comp_integral);    //X轴方向加速度对z轴的影响程度
  if(vxy[0] > 0.3 || vxy[0] < -0.3) {
    //x_comp_integral += _CtrlComp->cmpc_x_drag * pxy_err[0] * dtMPC / vxy[0];
    x_comp_integral += cmpc_x_drag * pz_err * dtMPC / vxy[0];
  }

  //printf("pz err: %.3f, pz int: %.3f\n", pz_err, x_comp_integral);
  // 最大求解次数，AMDD算法惩罚系数，收敛阈值，是否使用JCQP

  update_solver_settings(jcqp_max_iter, jcqp_rho,
      jcqp_sigma, jcqp_alpha, jcqp_terminate, use_jcqp);
  //t1.stopPrint("Setup MPC");

  // Timer t2;
  //cout << "dtMPC: " << dtMPC << "\n";
  update_problem_data_floats(p,v,q,w,r,yaw,weights,trajAll,alpha,mpcTable);
  //t2.stopPrint("Run MPC");
  //printf("MPC Solve time %f ms\n", t2.getMs());

  for(int leg = 0; leg < 4; leg++)
  {
    Vec3<float> f;
    for(int axis = 0; axis < 3; axis++)
      f[axis] = get_solution(leg*3 + axis);

    //printf("[%d] %7.3f %7.3f %7.3f\n", leg, f[0], f[1], f[2]);

    f_ff[leg] = -seResult.rBody * f;
    // Update for WBC
    Fr_des[leg] = f;
  }
}

// void ConvexMPCLocomotion::solveSparseMPC(int *mpcTable, CtrlComponents<float> &data) {
//   // X0, contact trajectory, state trajectory, feet, get result!
//   (void)mpcTable;
//   (void)data;
//   auto seResult = data.estimator->stateEstimate;

//   std::vector<ContactState> contactStates;
//   for(int i = 0; i < horizonLength; i++) {
//     contactStates.emplace_back(mpcTable[i*4 + 0], mpcTable[i*4 + 1], mpcTable[i*4 + 2], mpcTable[i*4 + 3]);
//   }

//   for(int i = 0; i < horizonLength; i++) {
//     for(u32 j = 0; j < 12; j++) {
//       _sparseTrajectory[i][j] = trajAll[i*12 + j];
//     }
//   }

//   Vec12<float> feet;
//   for(u32 foot = 0; foot < 4; foot++) {
//     for(u32 axis = 0; axis < 3; axis++) {
//       feet[foot*3 + axis] = pFoot[foot][axis] - seResult.position[axis];
//     }
//   }

//   _sparseCMPC.setX0(seResult.position, seResult.vWorld, seResult.orientation, seResult.omegaWorld);    //设置状态
//   _sparseCMPC.setContactTrajectory(contactStates.data(), contactStates.size());    //设置接触状态
//   _sparseCMPC.setStateTrajectory(_sparseTrajectory);
//   _sparseCMPC.setFeet(feet);
//   _sparseCMPC.run();

//   Vec12<float> resultForce = _sparseCMPC.getResult();

//   for(u32 foot = 0; foot < 4; foot++) {
//     Vec3<float> force(resultForce[foot*3], resultForce[foot*3 + 1], resultForce[foot*3 + 2]);
//     //printf("[%d] %7.3f %7.3f %7.3f\n", foot, force[0], force[1], force[2]);
//     f_ff[foot] = -seResult.rBody * force;
//     Fr_des[foot] = force;
//   }
// }
// 初始化稀疏矩阵求解
// void ConvexMPCLocomotion::initSparseMPC() {
//   Mat3<double> baseInertia;
//   baseInertia = robot_rs.I_body.template cast<double>();
//   // baseInertia << 0.07, 0, 0,
//   //             0, 0.26, 0,
//   //             0, 0, 0.242;
//   double mass = robot_rs.m;
//   double maxForce = 300;

//   std::vector<double> dtTraj;
//   for(int i = 0; i < horizonLength; i++) {
//     dtTraj.push_back(dtMPC);
//   }

//   Vec12<double> weights;
//   weights << 0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2;    //位置，速度，姿态角，姿态角速度
//   //weights << 0,0,0,1,1,10,0,0,0,0.2,0.2,0;

//   _sparseCMPC.setRobotParameters(baseInertia, mass, maxForce);    
//   _sparseCMPC.setFriction(0.4);
//   _sparseCMPC.setWeights(weights, 4e-5);      //设置目标约束的权重
//   _sparseCMPC.setDtTrajectory(dtTraj);       //时间轨迹，每次MPC的离散化时间

//   _sparseTrajectory.resize(horizonLength);
// }

