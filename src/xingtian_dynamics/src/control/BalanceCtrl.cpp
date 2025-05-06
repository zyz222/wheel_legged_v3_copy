/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "control/BalanceCtrl.h"
#include "common/mathTools.h"
#include "common/timeMarker.h"
template <typename T>
BalanceCtrl<T>::BalanceCtrl(T mass, Mat3<T> Ib, Mat6<T> S, T alpha, T beta)
            : _mass(mass), _Ib(Ib), _S(S), _alpha(alpha), _beta(beta){
    _Fprev.setZero();
    _g << 0, 0, -9.81;
    _fricRatio = 0.5;
    _fricMat <<  1,  0, _fricRatio,
                -1,  0, _fricRatio,
                 0,  1, _fricRatio,
                 0, -1, _fricRatio,
                 0,  0, 1;
}
//基于二次优化的方法来计算各腿的接触力。力分配控制！！！
//修改到这里了！！！
template <typename T>
BalanceCtrl<T>::BalanceCtrl(QuadrupedRobot<T> *robModel){
    Vec6<T> s;
    Vec12<T> w, u;

    _mass = robModel->getRobMass();     //机身质量
    _pcb = robModel->getPcb();           //机身人重心相对于机体的偏移
    _Ib = robModel->getRobInertial();   //机身惯性矩阵
    _g << 0, 0, -9.81;

    w << 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40;   //调整输入的权重参数

    u << 20, 20, 50, 20, 20, 50, 20, 20, 50, 20, 20, 50;            //二次项正则化参数。调大可以使输入更加平滑
    _alpha = 0.301;                    //权重W矩阵的系数
    _beta  = 0.5;                       //权重矩阵U的系数
    _fricRatio = 0.6;                   //摩擦锥系数

    s << 120, 120,120, 450, 450, 450;    //二次规划目标权重参数！   调节状态的，状态更逼近

    _S = s.asDiagonal();       //动力学方程的权重系数
    _W = w.asDiagonal();       //足端力大小的权重系数
    _U = u.asDiagonal();      //足端力改变的权重系数
    
    _Fprev.setZero();                 //上一时刻力向量初始化为0
    _fricMat <<  1,  0, _fricRatio,
                -1,  0, _fricRatio,
                 0,  1, _fricRatio,
                 0, -1, _fricRatio,
                 0,  0, 1;
}
// 计算足端接触力面的力向量，并返回
// feetPos2B: 四条腿相对于机体坐标系下的位置
// rotM: 机体系到机体坐标系旋转矩阵
// contact: 四条腿是否接触，1表示接触，0表示不接触
//ddpcd 期望加速度 
template <typename T> 
Vec34<T> BalanceCtrl<T>::calF(Vec3<T> ddPcd, Vec3<T> dWbd, RotMat<T> rotM, Vec34<T> feetPos2B, Vec4<T> contact){
    calMatrixA(feetPos2B, rotM, contact);  //计算接触力矩阵 A   这个也没问题！！！！
    calVectorBd(ddPcd, dWbd, rotM);       //计算期望力向量 期望位置，角速度，旋转矩阵 b
    calConstraints(contact);              //计算约束条件

    //加入系数的原因是不希望足端力出现图突变性，所以加入系数
    _G = _A.transpose()*_S*_A + _alpha*_W + _beta*_U;     //G
    _g0T = -_bd.transpose()*_S*_A - _beta*_Fprev.transpose()*_U;   //标准二次型的系数！！g

    solveQP();
    // std::cout << "接触状态: " << contact << std::endl;
    _Fprev = _F;
    // std::cout << "F: " << _F << std::endl;
    return vec12ToVec34(_F);
}
// 计算接触力矩阵
// 二次规划中的A矩阵
// feetPos2B: 四条腿相对于机体坐标系下的位置
// rotM: 机体系到机体坐标系旋转矩阵
// contact: 四条腿是否接触，1表示接触，0表示不接触
template <typename T> 
void BalanceCtrl<T>::calMatrixA(Vec34<T> feetPos2B, RotMat<T> rotM, Vec4<T> contact){
    for(int i(0); i < 4; ++i){
        _A.block(0, 3*i, 3, 3) = _I3;
        _A.block(3, 3*i, 3, 3) = skew<T>(feetPos2B.col(i) - rotM*_pcb);   //skew  构建反对称矩阵
    }
    // std::cout << "A: " << _A << std::endl;
}
// 计算期望力向量
// ddpcd 加速度  
// dWbd 期望角加速度
// rotM:机身当前姿态
//b 矩阵
template <typename T> 
void BalanceCtrl<T>::calVectorBd(Vec3<T> ddPcd, Vec3<T> dWbd, RotMat<T> rotM){
    _bd.head(3) = _mass * (ddPcd - _g);            //ma   这个没问题
    _bd.tail(3) = (rotM * _Ib * rotM.transpose()) * dWbd;   //RIRt * dWbd
    // std::cout << "B_: " << _bd << std::endl;   

}
//构建摩擦锥约束
//非接触腿的接触力设为0
//构建两个约束矩阵，CI和CE，CI为接触腿，CE为非接触腿
template <typename T> 
void BalanceCtrl<T>::calConstraints(Vec4<T> contact){
    int contactLegNum = 0;
    for(int i(0); i<4; ++i){
        if(contact(i) == 1){
            contactLegNum += 1;
        }
    }
    // std::cout << "contactLegNum: " << contactLegNum << std::endl;
    //动态调整约束矩阵大小
    //接触腿 12个力，每个腿有3个
    _CI.resize(5*contactLegNum, 12);
    _ci0.resize(5*contactLegNum);
    //非接触腿
    _CE.resize(3*(4 - contactLegNum), 12);
    _ce0.resize(3*(4 - contactLegNum));

    _CI.setZero();
    _ci0.setZero();       //接触腿的
    _CE.setZero();
    _ce0.setZero();

    int ceID = 0;   //非接触腿
    int ciID = 0;   //接触腿
    for(int i(0); i<4; ++i){
        if(contact(i) == 1){
            _CI.block(5*ciID, 3*i, 5, 3) = _fricMat;
            ++ciID;
        }else{
            _CE.block(3*ceID, 3*i, 3, 3) = _I3;
            ++ceID;
        }
    }
}
// 二次规划求解
template <typename T> 
void BalanceCtrl<T>::solveQP(){
    int n = _F.size();    //力矩阵
    int m = _ce0.size();   //非接触腿的约束条件向量
    int p = _ci0.size();   //接触腿的约束条件向量

    G.resize(n, n);

    CE.resize(n, m);
    CI.resize(n, p);

    g0.resize(n);
    ce0.resize(m);
    ci0.resize(p);

    x.resize(n);

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            G[i][j] = _G(i, j);
        }
    }  //所有的都输入给G矩阵

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            CE[i][j] = (_CE.transpose())(i, j);
        }  //m是非接触腿
    }

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < p; ++j) {
            CI[i][j] = (_CI.transpose())(i, j);
        }     //p是接触腿
    }

    for (int i = 0; i < n; ++i) {
        g0[i] = _g0T[i];
    }

    for (int i = 0; i < m; ++i) {
        ce0[i] = _ce0[i];
    }

    for (int i = 0; i < p; ++i) {
        ci0[i] = _ci0[i];
    }

    double value = solve_quadprog(G, g0, CE, ce0, CI, ci0, x);     //这个求解器还不能通用！

    for (int i = 0; i < n; ++i) {
        _F[i] = x[i];      //提取优化力的结果
    }
}
// template class BalanceCtrl<double>;
template class BalanceCtrl<float>;