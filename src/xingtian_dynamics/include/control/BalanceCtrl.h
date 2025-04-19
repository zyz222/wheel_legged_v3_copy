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
    Vec34<T> calF(Vec3<T> ddPcd, Vec3<T> dWbd, RotMat<T> rotM, Vec34<T> feetPos2B, VecInt4 contact);

private:
    void calMatrixA(Vec34<double> feetPos2B, RotMat<double> rotM, VecInt4 contact);
    void calVectorBd(Vec3<double> ddPcd, Vec3<double> dWbd, RotMat<double> rotM);
    void calConstraints(VecInt4 contact);
    void solveQP();

    Mat12<double> _G, _W, _U;
    Mat6<double> _S;
    Mat3<double> _Ib;
    Vec6<double> _bd;
    Vec3<double> _g;
    Vec3<double> _pcb;
    Vec12<double> _F, _Fprev, _g0T;
    double _mass, _alpha, _beta, _fricRatio;
    Eigen::MatrixXd _CE, _CI;
    Eigen::VectorXd _ce0, _ci0;
    Eigen::Matrix<double, 6 , 12> _A;
    Eigen::Matrix<double, 5 , 3 > _fricMat;

    quadprogpp::Matrix<double> G, CE, CI;
    quadprogpp::Vector<double> g0, ce0, ci0, x;


};

#endif  // BALANCECTRL_H