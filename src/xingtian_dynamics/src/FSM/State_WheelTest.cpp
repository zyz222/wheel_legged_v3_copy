/**********************************************************************
 Copyright (c) 2020-2023, zyz. All rights reserved.
***********************************************************************/

#include "FSM/State_WheelTest.h"
// #include "qpOASES.hpp"

#include "Eigen/Dense"
#include "Eigen/Sparse"

using namespace std;
using namespace Eigen;
template <typename T>
State_WheelTest<T>::State_WheelTest(CtrlComponents<T> *ctrlComp)
    :FSMState<T>(ctrlComp,FSMStateName::WHEELTEST, "wheelTest"),
    _est(ctrlComp->estimator), _robModel(ctrlComp->robotModel), 
    _balCtrl(ctrlComp->balCtrl), _contact(ctrlComp->contact){

    initRecv();
    _xMax = 0.05;
    _xMin = -_xMax;
    _yMax = 0.03;
    _yMin = -_yMax;
    _zMax = 0.15;
    _zMin = 0.0;
    _yawMax = M_PI / 180;
    _yawMin = -_yawMax;
    _Kpp = Vec3<T>(50, 0.1, 50).asDiagonal();    //Kp系数
    _Kdp = Vec3<T>(5, 0.1, 5).asDiagonal();      // Kd系数
    _kpw = 50;                          //机身角速度Kp
    _Kdw = Vec3<T>(30, 30, 30).asDiagonal();    // kd

    /*位置环控制用*/
    _rowMax = 50 * M_PI / 180;
    _rowMin = -_rowMax;
    _pitchMax = 50 * M_PI / 180;
    _pitchMin = -_pitchMax;
    _yawMax_p = 150 * M_PI / 180;
    _yawMin_p = -_yawMax;
    _heightMax = 0.2;   //m
    _heightMin = -0.1;    //m   
    
}
template <typename T>
void State_WheelTest<T>::mpc_init()     //这个是绝对控制量MPC，还有一种是增量式MPC
{
    n = 5;  //轮式运动控制MPC状态维度
    p = 4;  //轮子运动控制MPC控制输入维度
    Np = 5; //MPC预测步数
    dt = 0.1; ///MPC采样时间间隔
    theta_k = 0; // 初始偏航角 状态变量
    x,y = 0;     //状态变量初始化
    v,omega = 0;  //状态变量初始化
    f1,f2,f3,f4 = 0;//控制变量初始化
    m = _robModel->getRobMass();      //机身质量
    J = _robModel->getRobInertial()(2,2);   //绕Z轴转动惯量
    L = 0.4;   //轮距宽0.4m
    A_state.setIdentity();       //初始化A矩阵    A矩阵在运行过程中需要实时更新！！
    B_ctrl<< 0, 0, 0, 0,       //初始化B矩阵
         0, 0, 0, 0,
         0, 0, 0, 0,
         dt / m, dt / m, dt / m, dt / m,
         (-dt * L) / (2 * J), (-dt * L) / (2 * J), (dt * L) / (2 * J), (dt * L) / (2 * J);
    A_powers.resize(Np);       //预计算A的N次幂矩阵
    Phi = MatrixXf::Zero(n*Np,n);         //填充fai矩阵
    Gamma = MatrixXf::Zero(n * Np, p * Np);
    // /*
    // Phi.block(0, 0, n, n) = A;
    // A_powers[0] = A; // A^1
    // for (int k = 1; k < Np; ++k) {
    //     A_powers[k] = A_powers[k-1] * A; // A^{k+1} = A^k * A
    //     Phi.block(k * n, 0, n, n) = A_powers[k];
    // }
    
    //  */
    
    //权重矩阵
    
    S.setIdentity();
    S(0,0) = 100;
    S(1,1) = 100;
    S(2,2) = 500;
    S(3,3) = 500;
    S(4,4) = 500;
    
    Q.setIdentity();
    Q(0,0) = 100000;
    Q(1,1) = 100000;
    Q(2,2) = 50000;
    Q(3,3) = 50000;
    Q(4,4) = 50000;
    
    R.setIdentity();
    R(0,0) = 50;
    R(1,1) = 50;
    R(2,2) = 50;
    R(3,3) = 50;
   
    // 构造 \Omega = diag(Q, Q, ..., Q, S) 大小 (n*Np) x (n*Np) 不用更新！！
    Omega = MatrixXf::Zero(n*Np, n*Np);
    for(int i=0; i<Np; ++i) {
        // 对角块依次放 Q，最后一块可以放 S
        // 这里演示：前 Np-1 个都是 Q，第 Np 个放 S
        if (i < Np-1) {
            Omega.block(i*n, i*n, n, n) = Q;
        } else {
            Omega.block(i*n, i*n, n, n) = S;
        }
    }
    // 构造 \Psi = diag(R, R, ..., R) 大小 (p*Np) x (p*Np)
    Psi = MatrixXf::Zero(p*Np, p*Np);                //不用更新！
    for(int i=0; i<Np; ++i) {
        Psi.block(i*p, i*p, p, p) = R;
    }
    // ===================== 5. 形成 QP 的 Hessian 矩阵 H 和线性项 F =====================
    //    J(U) = 1/2 ( (Phi*x - Xref) + Gamma*U )^T Omega (...) + 1/2 (U - Uref)^T Psi (..)
    //    => H = Gamma^T * Omega * Gamma + Psi
    //       f = Gamma^T * Omega * (Phi*x - Xref) - Psi * Uref
     //状态空间
    // Vec5 x0;      //状态向量   
    // x0<< x, y,theta_k ,v, omiga;

    // Vec4 u0;     //控制向量
    // u0<< f1, f2, f3, f4;
    x0 = VectorXf::Zero(n);               // 当前状态
    Xref = VectorXf::Zero(n*Np);          // 期望轨迹
    Uref = VectorXf::Zero(p*Np);          // 期望控制序列
    // 在实际代码中你会有更具体的参考轨迹 Xref、参考控制 Uref。

    x0<< x,y,theta_k,v,omega;
    Xref.setZero();     //后续需要更新的
    Uref.setZero();   
    // ===================== 6. 构造 QP 的约束 =====================
    // 一般形式:
    //    minimize  1/2 U^T H U + f^T U
    // subject to: lbA <= A_ineq * U <= ubA,
    //             lb  <= U         <= ub.
    //

    // 演示：假设 U 每个分量 ∈ [0, 10]，车轮半径0.08m
    lbU = VectorXd::Ones(p*Np)*-70.0;                //控制输入下限
    ubU = VectorXd::Ones(p*Np)*70.0;              //控制输入上限

}
template <typename T>
void State_WheelTest<T>::update_mpc_state(){
    // 更新状态
    // 更新约束
    // 更新目标
    // 更新权重
    // 更新预测模型
    // 更新控制器
    // 更新控制器参数
    // 更新控制器状态
    // 更新控制器输出
    // 更新控制器输入
    double vx,vy,vz;
    x,y,z = _posBody(0),_posBody(1),_posBody(2);     //机身位置
    vx,vy,vz = _velBody(0),_velBody(1),_velBody(2);  //机身速度
    v = std::sqrt(vx*vx+vy*vy); //机身速度大小 
    theta_k = this->_lowState->getYaw();
    omega = this->_lowState->getDYaw(); //机身角速度
    x0<<x,y,theta_k,v,omega;       //最新的状态
    //更新状态A矩阵
    A_state(0,3)= dt*cos(theta_k); 
    // A_state[0][3]= T*cos(theta_k);
    A_state(1,3)= dt*sin(theta_k);
    A_state(2,4) = dt;
    // // A_state(0,3)= 100; 
    // std::cout << "T: " << T << std::endl;
    // std::cout << "COS(theta_k): " << cos(theta_k) << "k:"<< theta_k << std::endl;
    // //更新预测矩阵
    // std::cout << "A(0,3): " << A_state(0,3) << std::endl;
    // // std::cout << "A[]: " << A_state[0][3] << std::endl;
    // std::cout << "B: " << B_ctrl << std::endl;
    A_powers[0] = A_state; // A^1
    for (int k = 1; k < Np; ++k) {
        A_powers[k] = A_powers[k-1] * A_state; // A^{k+1} = A^k * A
    }
    // std::cout << "A_powers: " << A_powers[0] << std::endl;
    for (int k = 0; k < Np; ++k) {
        Phi.block(k * n, 0, n, n) = A_powers[k];
    }

    // 构建 Γ 矩阵 (n*N_p x p*N_p)
   
    for (int i = 0; i < Np; ++i) {         // 行块索引
        for (int j = 0; j <= i; ++j) {      // 列块索引 (仅下三角)
            int power = i - j;              // A^{i-j}
            if (power == 0) {               //对角线都是B矩阵
                A_power_B = B_ctrl;              // A^0 * B = B
            } else {
                // 计算 A^{power} * B
                A_power_B = A_powers[power - 1] * B_ctrl; // A_powers[power-1] = A^power
            }
            // 填充到 Γ 的对应块
            Gamma.block(i * n, j * p, n, p) = A_power_B;
        }
    }
    // std::cout << "Phi: " << Phi << std::endl;
    // std::cout << "Gamma: " << Gamma << std::endl;


}
// 这个函数后期将会删除，更换新的指令
template <typename T>
void State_WheelTest<T>::update_trajectory(){

    // 更新初始控制输入
    double desired_x,desired_y,desired_theta,desired_v,desired_w;
    // 更新参考轨迹
    desired_w = _dYawCmd ;
    if(_vCmdBody(0)>=0){
        desired_v =std::sqrt(_vCmdBody(0)*_vCmdBody(0)+_vCmdBody(1)*_vCmdBody(1));
        if(_vCmdBody(1)>=0){
            desired_theta = _dYawCmd*dt + this->_lowState->getYaw()+std::atan2(_vCmdBody(1),_vCmdBody(0));     //当前航向角+输入的期望角度 + 速度计算出来的航向角
        }else{
            desired_theta = _dYawCmd*dt + this->_lowState->getYaw()-std::atan2(_vCmdBody(1),_vCmdBody(0));
        }
        
    }else{
        desired_v =-std::sqrt(_vCmdBody(0)*_vCmdBody(0)+_vCmdBody(1)*_vCmdBody(1));
        if(_vCmdBody(1)>=0){
            desired_theta = _dYawCmd*dt + this->_lowState->getYaw()+M_PI-std::atan2(_vCmdBody(1),_vCmdBody(0));
        }else{
            desired_theta = _dYawCmd*dt + this->_lowState->getYaw()+M_PI+std::atan2(_vCmdBody(1),_vCmdBody(0));
        }
        
    }
    // std::cout<<"desired_theta:"<<desired_theta<<std::endl;
    desired_x = _est->cheater_getPosition()(0)+_vCmdBody(0)*dt;
    desired_y = _est->cheater_getPosition()(1)+_vCmdBody(1)*dt;
    // 期望轨迹
    for(int i=0;i<Np;++i)
    {
        int startIndex = i * n; // 计算当前步的起始索引
        Xref.segment(startIndex, n) << desired_x, desired_y, desired_theta, desired_v, desired_w;
        // std::cout << "Xref: " << Xref.transpose() << std::endl;
        // 参考轨迹这里更新的没问题！！
    }
   
}
template <typename T>
void State_WheelTest<T>::enter(){
    _pcdInit = _est->cheater_getPosition();
    _pcdInit(2) = 0.15;
    _pcd = _pcdInit;
    _RdInit = this->_lowState->getRotMat();
    mpc_init();
    this->_ctrlComp->setAllStance();              //重置所有腿为触地状态
    this->_ctrlComp->ioInter->zeroCmdPanel();  //用户输入面板


    /*位置控制环 */
    for(int i = 0; i < 4; i++)
    {
        this->_lowCmd->setSimFreeStanceGain(i);
         
        this->_lowCmd->setZeroDq(i);
        this->_lowCmd->setZeroTau(i);
    }
    _initVecOX = this->_ctrlComp->robotModel->getX(*this->_lowState);     //获取0号腿的足端位置
    _initVecXP = this->_ctrlComp->robotModel->getVecXP(*this->_lowState);  //获取其他腿相对于0号腿的足端位置，在世界坐标系下
    this->_ctrlComp->setAllStance();
    this->_lowCmd->setWheelRollGain();
    _tau.setZero();

}
//这里要进行修改，控制车轮的转矩！！！！一部分控制平衡，一部分控制轮子！！！！
template <typename T>
void State_WheelTest<T>::run(){
    this->_userValue = this->_lowState->userValue;  //获取用户平衡控制输入值
    std::cout<< "pcd" <<_pcd<<std::endl;
    _pcd(0) = _pcdInit(0) + invNormalize(this->_userValue.ly, _xMin, _xMax);      // 机器人重心位置 x
    _pcd(1) = _pcdInit(1) - invNormalize(this->_userValue.lx, _yMin, _yMax);     // 重心位置y
    _pcd(2) = _pcdInit(2) + invNormalize(this->_userValue.ry, _zMin, _zMax);     // 重心位置z
    double yaw = invNormalize(this->_userValue.rx, _yawMin, _yawMax);    //期望航向角,之后可以改到轮子上！！
    _Rd = rpyToRotMat<T>(0.0, 0.0, yaw)*_RdInit;   //期望的航向角度，先将机器人绕z轴旋转yaw角度，再利用初始的旋转姿态角度
                                            // _Rd指的是期望与现在的机体角度的偏差！！！也就是目标姿
    _posBody = _est->cheater_getPosition();     //获取当前机身位置,世界坐标系
    _velBody = _est->cheater_getVelocity();     //获取当前机身速度   
    _B2G_RotMat = this->_lowState->getRotMat();    //获取机身到世界坐标系的旋转矩阵
    _G2B_RotMat = _B2G_RotMat.transpose();    //世界坐标系到机身的旋转矩阵
    getUserCmdwheel();                     //获取用户指令 车轮控制输入值vx vy wz
    WheelControl();    //计算车轮与地面的摩擦力
    /*calcTau();            //计算关节扭矩 和车轮扭矩

    
    this->_lowCmd->setWheelRollGain();
    this->_lowCmd->setTau(_tau);     //设置关节力矩
    this->_lowCmd->setQ(_q);       //设置关节角度,也就是获取当前的关节角度,力矩控制！！
    */

    /*关节位置控制用！！ */
    Vec34<T> vecOP;
    vecOP = _calcOP( invNormalize(this->_userValue.lx, _rowMin, _rowMax),     //归一化输入
                     invNormalize(this->_userValue.ly, _pitchMin, _pitchMax),
                    -invNormalize(this->_userValue.rx, _yawMin, _yawMax),
                     invNormalize(this->_userValue.ry, _heightMin, _heightMax) );
    _calcCmd(vecOP);
    
    wheel_nums = 4;
    friction_torque = wheel_r*mu*m*g/wheel_nums;   //4是轮子个数
    friction_torque = 0.00*friction_torque;
    for(int i = 0; i < 4; i++)
    {
        _tau(3 * i + 2) = U_opt(i)*wheel_r + friction_torque;  //加上摩擦力矩 LF-LR-RR-RF
    }
    this->_lowCmd->setTau(_tau);

}
//车轮控制逻辑！！！根据动力学模型！！！！这里可以通过MPC来进行计算！！轮式动力学模型！！
template <typename T>
void State_WheelTest<T>::WheelControl(){
    //当前车辆位置 _posBody(x,y,z)
   //根据_vCmdBody（vx，vy，wz） 来计算车轮的扭矩，要转换乘期望轨迹
   //当前的车辆速度 _velBody(x,y,z)     Vec3
   //当前车辆角速度 this->_lowState->getGyroGlobal()    获取全局坐标系下的角速度（xyz）
   update_mpc_state();
//    std::cout<<"更新状态之前没问题"<<std::endl;
   update_trajectory();
//    std::cout<<"更新轨迹之前没问题"<<std::endl;
    //求解最优控制输入
    // 先计算 (Phi*x0 - Xref)
    diffX = Phi * x0 - Xref;   // 大小 = n*Np   误差项
    // std::cout<<"diffX = "<<diffX<<std::endl;
    // 计算 H
    H = (Gamma.transpose() * Omega * Gamma + Psi).template cast<double>();  // 大小 = (p*Np) x (p*Np)    这个没问题，
    // 计算 f
    f = (Gamma.transpose() * Omega * diffX).template cast<double>(); // 大小 = p*Np   //////反馈的状态量为diffX，
    // std::cout<<"代价函数之前没问题"<<std::endl;
    // std::cout<< "Psi矩阵是否全零？\n" << (Psi.isZero() ? "是" : "否") << std::endl;
    // std::cout << "Gamma矩阵示例：\n" << Gamma.block(0,0,4,4) << std::endl;
    // std::cout << "Omega矩阵示例：\n" << Omega.block(0,0,4,4) << std::endl;
    // VectorXd f_debug = Gamma.transpose() * Omega * diffX;
    // std::cout << "f向量是否全零？\n" << (f_debug.isZero() ? "是" : "否") << std::endl;
    // std::cout << "H矩阵是否正定？\n" << (H.llt().info() == Eigen::Success ? "是" : "否") << std::endl;
    // std::cout << "f向量示例： " << f.head(4).transpose() << std::endl;
    // std::cout << "H矩阵前4x4块：\n" << H.block(0,0,4,4) << std::endl;
    // std::cout << "f向量前4元素： " << f.head(4).transpose() << std::endl;
    // std::cout << "Gamma^T * Omega * diffX 前4元素： " << (Gamma.transpose() * Omega * diffX).head(4).transpose() << std::endl;
     // ===================== 6. 调用 OSQP 求解 =====================
    // 转换为 OSQP 数据格式
    A_eq.setIdentity(p*Np, p*Np);      //不等式矩阵
    H_sparse = H.template triangularView<Eigen::Upper>().toDenseMatrix().sparseView();      //转换为稀疏矩阵的格式
    A_eq_sparse = A_eq.sparseView();     
              
    H_sparse.makeCompressed();     //确保是CSC的压缩格式
    A_eq_sparse.makeCompressed();
    // std::cout<<"求解器之前没问题"<<std::endl;
    std::vector<OSQPInt> H_outer(H_sparse.outerIndexPtr(), 
                                H_sparse.outerIndexPtr() + H_sparse.outerSize() + 1);
    std::vector<OSQPInt> H_inner(H_sparse.innerIndexPtr(),
                                H_sparse.innerIndexPtr() + H_sparse.nonZeros());

    std::vector<OSQPInt> A_outer(A_eq_sparse.outerIndexPtr(),
                                A_eq_sparse.outerIndexPtr() + A_eq_sparse.outerSize() + 1);
    std::vector<OSQPInt> A_inner(A_eq_sparse.innerIndexPtr(),
                                A_eq_sparse.innerIndexPtr() + A_eq_sparse.nonZeros());

    // 构造P矩阵
    
    P_csc.m = H_sparse.rows();
    P_csc.n = H_sparse.cols();
    P_csc.p = H_outer.data();
    P_csc.i = H_inner.data();
    P_csc.x = H_sparse.valuePtr();
    P_csc.nz = -1;
    P_csc.nzmax = H_sparse.nonZeros();
    
    // 构造A矩阵
    
    A_csc.m = A_eq_sparse.rows();
    A_csc.n = A_eq_sparse.cols();
    A_csc.p = A_outer.data();
    A_csc.i = A_inner.data();
    A_csc.x = A_eq_sparse.valuePtr();
    A_csc.nz = -1;
    A_csc.nzmax = A_eq_sparse.nonZeros();

    F = f.data();
    l = lbU.data();
    u = ubU.data();
    solver  = nullptr;
    settings = (OSQPSettings*)malloc(sizeof(OSQPSettings));    //分配内存以存储OSQP的设置参数
    if (!settings) {
        std::cerr << "Failed to allocate memory for settings!" << std::endl;
    }   
    osqp_set_default_settings(settings);                                      //设置默认的设置参数
    settings->verbose = true;                                                 //设置为true，以便在求解过程中输出详细信息 ，不需要就换成false    

    OSQPInt exitflag = osqp_setup(
    &solver,
    &P_csc,         // P矩阵
    F,        // q向量（线性项）
    &A_csc,         // A矩阵（约束）
    l,              // 下界
    u,              // 上界
    A_eq_sparse.rows(), // 约束数量
    H_sparse.cols(),    // 变量数量
    settings        // 设置参数
    );

    // std::cout<<"求解器设置没问题"<<std::endl;
    if (exitflag != 0) {
    std::cerr << "OSQP初始化失败，错误码：" << exitflag << std::endl;
    free(settings);
    return;
    }
    // 求解
    // 求解问题
    osqp_solve(solver);
    // std::cout<<"求解没问题"<<std::endl;
    /// 提取最优解
    U_opt = Map<VectorXd>(solver->solution->x, p*Np);// 确保维度正确

    // 输出第一个控制量
    std::cout << "Optimal Forces (F1-F4): " 
            << U_opt.segment(0, 4).transpose() << std::endl;
    // 释放内存
   // 清理资源
    



}
template <typename T>
void State_WheelTest<T>::exit(){
    this->_ctrlComp->ioInter->zeroCmdPanel();
    osqp_cleanup(solver);
    free(settings);

}
//检查状态切换条件
template <typename T>
FSMStateName State_WheelTest<T>::checkChange(){
    if(this->_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(this->_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else if(this->_lowState->userCmd == UserCommand::L2_X){
        return FSMStateName::FREESTAND;
    }
    else{
        return FSMStateName::WHEELTEST;
    }
}
template <typename T>
void State_WheelTest<T>::calcTau(){

    _ddPcd = _Kpp*(_pcd - _posBody) + _Kdp * (Vec3<T>(0, 0, 0) - _velBody);   //重心偏移需要的加速度,这个看起来没啥问题！！
    // 下边这个是姿态！这个是重心位置的差
    // std::cout << "ddPcd: " << _ddPcd << std::endl;

    _dWbd  = _kpw*rotMatToExp<T>(_Rd*_G2B_RotMat) + _Kdw * (Vec3<T>(0, 0, 0) - this->_lowState->getGyroGlobal());  //重心偏移需要的角速度
    // _dWbd =  Vec3(0, 0, 0);  //这个是姿态角度的差
  
    _posFeet2BGlobal = _est->getPosFeet2BGlobal();   //这个获取全局坐标系下的坐标，也没问题！！！
    
    _forceFeetGlobal = - _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact);
  
    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;    //将足端力从全局坐标系转换到机体坐标系
 
    _q = vec34ToVec12(this->_lowState->getQ());    //获取关节角度
 
    _tau = _robModel->getTau(_q, _forceFeetBody);   //静力学计算关节力矩，腿部关节的力矩
    
    wheel_nums = 4;
    friction_torque = wheel_r*mu*m*g/wheel_nums;   //4是轮子个数
    friction_torque = 0.002*friction_torque;
    for(int i = 0; i < 4; i++)
    {
        _tau(3 * i + 2) = U_opt(i)*wheel_r + friction_torque;  //加上摩擦力矩 LF-LR-RR-RF
    }
}

template <typename T>
void State_WheelTest<T>::setHighCmd(T vx, T vy, T wz){
    //
    _vCmdBody(0) = -vx;
    _vCmdBody(1) = vy;
    _vCmdBody(2) = 0; 
    _dYawCmd = -wz;

}
template <typename T>
void State_WheelTest<T>::getUserCmdwheel(){    //获取车轮的控制指令
    setHighCmd(_vx, _vy, _wz);             //输入高级控制命令
    ros::spinOnce();    //处理完一次消息后立即返回
}
template <typename T>
void State_WheelTest<T>::twistCallback(const geometry_msgs::Twist& msg){
    _vx = msg.linear.x;
    _vy = msg.linear.y;
    _wz = msg.angular.z;
}
template <typename T>
void State_WheelTest<T>::initRecv(){    //订阅速度指令
    _cmdSub = _nm.subscribe("/cmd_vel", 1, &State_WheelTest::twistCallback, this);
}
template <typename T>
void State_WheelTest<T>::_calcCmd(Vec34<T> vecOP){
    Vec12<T> q;
    Vec34<T> WHEEL;
    q = this->_ctrlComp->robotModel->getQ(vecOP, FrameType::BODY); 
    WHEEL = this->_ctrlComp->lowState->getQ();
    q(2) = WHEEL(2,0);
    q(5) = WHEEL(2,1);
    q(8) = WHEEL(2,2);
    q(11) = WHEEL(2,3);
    // std::cout << "q_final: " << q << std::endl;

    // std::cout << "state_q: " << this->_ctrlComp->lowState->getQ() << std::endl;
    this->_lowCmd->setQ(q);              //只有前两行的角度是hip和knee,而忽略了第三行wheel
}
template <typename T>
Vec34<T> State_WheelTest<T>::_calcOP(T row, T pitch, T yaw, T height){
    Vec3<T> vecXO = -_initVecOX;           //负的0号腿的足端位置,机体坐标系下，相反表示重心在左前脚下的位置
    // std::cout << "vecXO: " << vecXO<< std::endl;
    // std::cout << "VECXP: " << _initVecXP << std::endl;
    vecXO(2) -= height;                  //高度

    RotMat<T> rotM = rpyToRotMat<T>(row, pitch, yaw);

    HomoMat<T> Tsb = homoMatrix<T>(vecXO, rotM);       //机器人重心到基坐标系的变换矩阵
    HomoMat<T> Tbs = homoMatrixInverse<T>(Tsb);

    Vec4<T> tempVec4;
    Vec34<T> vecOP;
    for(int i(0); i<4; ++i){
        tempVec4 = Tbs * homoVec<T>(_initVecXP.col(i));
        vecOP.col(i) = noHomoVec<T>(tempVec4);       
    } 
  
    return vecOP;     //获取到4个足端的位置
}
// template class State_WheelTest<double>;
template class State_WheelTest<float>;