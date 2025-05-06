/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef BALANCECTRL_H
#define BALANCECTRL_H

#include "common/cppTypes.h"
#include "thirdParty/quadProgpp/QuadProg++.hh"
#include "common/xingtianrobot.h"


template <typename T>
class BalanceCtrl{
public:
    BalanceCtrl(T mass, Mat3<T> Ib, Mat6<T> S, T alpha, T beta);
    BalanceCtrl(QuadrupedRobot<T> *robModel);
    Vec34<T> calF(Vec3<T> ddPcd, Vec3<T> dWbd, RotMat<T> rotM, Vec34<T> feetPos2B, Vec4<T> contact);

private:
    void calMatrixA(Vec34<T> feetPos2B, RotMat<T> rotM, Vec4<T> contact);
    void calVectorBd(Vec3<T> ddPcd, Vec3<T> dWbd, RotMat<T> rotM);
    void calConstraints(Vec4<T> contact);
    void solveQP();

    Mat12<float> _G, _W, _U;
    Mat6<float> _S;
    Mat3<float> _Ib;
    Vec6<float> _bd;
    Vec3<float> _g;
    Vec3<float> _pcb;
    Vec12<float> _F, _Fprev, _g0T;
    float _mass, _alpha, _beta, _fricRatio;
    Eigen::MatrixXf _CE, _CI;
    Eigen::VectorXf _ce0, _ci0;
    Eigen::Matrix<float, 6 , 12> _A;
    Eigen::Matrix<float, 5 , 3 > _fricMat;

    quadprogpp::Matrix<double> G, CE, CI;
    quadprogpp::Vector<double> g0, ce0, ci0, x;


};

#endif  // BALANCECTRL_H