/**********************************************************************
 Copyright (c) 2020-2023, zyz. All rights reserved.
***********************************************************************/
#include <stdio.h>
#include <string>
#include <vector>
#include <stdexcept>
#include "common/xingtianrobot.h"
#include <iostream>
#include "common/orientation_tools.h"
#include "Utilities/Utilities_print.h"
using namespace  std;
// template class SpatialInertia<double>;
template <typename T>
//获取0号腿的足端位置，在机体坐标下XYZ
Vec3<T> QuadrupedRobot<T>::getX(LowlevelState<T> &state){
    return getFootPosition(state, 0, FrameType::BODY);
}
//获取其它足端位置相对于左前腿的位置
template <typename T>
Vec34<T> QuadrupedRobot<T>::getVecXP(LowlevelState<T> &state){
    Vec3<T> x = getX(state); //机体系下足端位置 左前腿
    // std::cout<<"机体坐标系下左前腿位置: "<<x<<std::endl;
    Vec34<T> vecXP, qLegs;
    qLegs = state.getQ();   //第一个关节角度是车轮的 LF LR RR RF HIP KNEE WHEEL

    for(int i(0); i < 4; ++i){
        vecXP.col(i) = _Legs[i]->calcPEe2B(qLegs.col(i)) - x;
    }
    return vecXP;
}
// Inverse Kinematicsw
//根据足端位置获取关节角度
template <typename T>
Vec12<T> QuadrupedRobot<T>::getQ(const Vec34<T> &vecP, FrameType frame){
    Vec12<T> q;   //向量
    for(int i(0); i < 4; ++i){
        // std::cout<<"第i条腿超出范围: "<<i<<std::endl;
        q.segment(3*i, 3) = _Legs[i]->calcQ(vecP.col(i), frame);
    }
    return q;
}
//获取关节角速度
//vel 是车轮末端期望速度
//返回值是关节角速度
template <typename T>
Vec12<T> QuadrupedRobot<T>::getQd(const Vec34<T> &pos, const Vec34<T> &vel,LowlevelState<T> &state, FrameType frame){
    Vec12<T> qd;     //向量
    Vec34<T> _qd = state.getQd();
    for(int i(0); i < 4; ++i){
        // calQd是输入三个位置，三个速度,三个原始的角速度
        qd.segment(3*i, 3) = _Legs[i]->calcQd(pos.col(i), vel.col(i),_qd.col(i),frame);
    }
    return qd;
}
//输入关节角度和接触力
//返回关节力矩，足端力 x 和 z
template <typename T>
Vec12<T> QuadrupedRobot<T>::getTau(const Vec12<T> &q, const Vec34<T> feetForce){
    Vec12<T> tau;
    for(int i(0); i < 4; ++i){
        tau.segment(3*i, 3) = _Legs[i]->calcTau(q.segment(3*i, 3), feetForce.col(i));
    }
    return tau;
}

// Forward Kinematics
// 获取足端位置，相对于机体或者髋关节
//返回值是三维向量(x,z,y)，输入二维(q1,q2)，调用的话，需要循环4次
template <typename T>
Vec3<T> QuadrupedRobot<T>::getFootPosition(LowlevelState<T> &state, int id, FrameType frame){
    Vec34<T> qLegs= state.getQ();    //获取当前关节角度 LF LR RR RF HIP KNEE WHEEL   这个没问题
    // std::cout<<"qLegs: "<<qLegs<<std::endl;
    if(frame == FrameType::BODY)
    {   
        return _Legs[id]->calcPEe2B(qLegs.col(id));
    }
    else if(frame == FrameType::HIP)
    {
        return _Legs[id]->calcPEe2H(qLegs.col(id));
    }else
    {
        std::cout << "[ERROR] The frame of function: getFootPosition can only be BODY or HIP." << std::endl;
        exit(-1);
    }
}

// Forward derivative Kinematics
// 根据关节角度获取足端速度
template <typename T>
Vec3<T> QuadrupedRobot<T>::getFootVelocity(LowlevelState<T> &state, int id){
    Vec34<T> qLegs = state.getQ(); //获取原始关节角度
    Vec34<T> qdLegs= state.getQd(); //获取原始的关节角速度
    Vec3<T> footvelocity;
    footvelocity = _Legs[id]->calcVEe(qLegs.col(id), qdLegs.col(id));
    return footvelocity;
}

// Forward Kinematics
// 获取足端位置，输入关节角度
template <typename T>
Vec34<T> QuadrupedRobot<T>::getFeet2BPositions(LowlevelState<T> &state, FrameType frame){
    Vec34<T> feetPos; //存储三行四列所有足端位置
    if(frame == FrameType::GLOBAL){
        for(int i(0); i<4; ++i){
            feetPos.col(i) = getFootPosition(state, i, FrameType::BODY); 
        }
        feetPos = state.getRotMat() * feetPos;    //从机身坐标系下转换到世界坐标系下！！！
    }
    else if((frame == FrameType::BODY) || (frame == FrameType::HIP)){
        for(int i(0); i<4; ++i){
            feetPos.col(i) = getFootPosition(state, i, frame);
        }
    }
    else{
        std::cout << "[ERROR] Frame error of function getFeet2BPositions" << std::endl;
        exit(-1);
    }
    return feetPos;
}
//在这里进行修改，车轮的速度
template <typename T>
Vec34<T> QuadrupedRobot<T>::getFeet2BVelocities(LowlevelState<T> &state, FrameType frame){
    Vec34<T> feetVel;
    for(int i(0); i<4; ++i){
        feetVel.col(i) = getFootVelocity(state, i); //获取足端速度 J*dq  x y z 
    }

    if(frame == FrameType::GLOBAL){
        Vec34<T> feetPos = getFeet2BPositions(state, FrameType::BODY);   //获取机体系下足端位置
        feetVel += skew(state.getGyro()) * feetPos; //v + 【w】x P     机身惯性坐标下足端速度
        return state.getRotMat() * feetVel;        //世界坐标系下足端相对于机身的速度
    }
    else if((frame == FrameType::BODY) || (frame == FrameType::HIP)){
        return feetVel;
    }
    else{
        std::cout << "[ERROR] Frame error of function getFeet2BVelocities" << std::endl;
        exit(-1);
    }   
}
template <typename T>
Mat3<T> QuadrupedRobot<T>::getJaco(LowlevelState<T> &state, int legID){
    return _Legs[legID]->calcJaco(state.getQ().col(legID));
}
// 投影足端位置到可达范围内
template <typename T>
void QuadrupedRobot<T>::projectToReachableRange(Vec3<T> &pos) {
    for (int i = 0; i < 4; ++i) {
        if (i < 2)
       {
        pos(1) = 0.201;
       }else{
        pos(1) = -0.201;
       }

       pos = pos - (_Legs[i]->getHip2B()); 
       
    }
    T x = pos(0);
    T z = pos(2);
    T r_max = 0.28;
    T r_min = 0.0;
    // 计算当前距离
    T r = std::sqrt(x * x + z * z);
    // std::cout
    // 如果超出最大可达距离，投影到最大可达距离
    if (r > r_max) {
        // T scale = r_max / r;
        pos(0) = x /r  * r_max;
        pos(2) = z /r  * r_max;
    }

    // 如果小于最小可达距离，投影到最小可达距离
    // if (r < r_min) {
    //     // T scale = r_min / r;
    //     pos(0) = x /r * r_min;
    //     pos(2) = z /r * r_min;
    // }
}


//计算惯性矩阵
template <typename T>
Mat3<T> QuadrupedRobot<T>::calcInertialMatrix(const Vec3<T> &theta){
    // std::cout<<"inertiamatrix_1"<<std::endl;
    
    T c1 = std::cos(theta(0));
    T c2 = std::cos(theta(1));
    T s1 = std::sin(theta(0));
    T s2 = std::sin(theta(1));
    T c12 = c1*c2 - s1*s2;
    // T c12 = std::cos(theta(0) + theta(1));
    T s12 = s1*c2 + c1*s2;
    // T s12 = std::sin(theta(0) + theta(1));
    m11 = Iyy1 + Iyy2 + Iyy3 
          + wheel_mass * (L2 * c12 + L1 * c1) * (L2 * c12 + L1 * c1)
          + knee_mass * (Lc2 * c12 + L1 * c1) * (Lc2 * c12 + L1 * c1)
          + wheel_mass * (L2 * s12 + L1 * s1) * (L2 * s12 + L1 * s1)
          + knee_mass * (Lc2 * s12 + L1 * s1) * (Lc2 * s12 + L1 * s1)
          + hip_mass * Lc1 * Lc1;

    m12 = Iyy2 + Iyy3 + wheel_mass *L2 *s12 * (L2 * s12 + L1 * s1)
          + Lc2 * knee_mass * s12 * (Lc2 * s12 + L1 * s1)
          + wheel_mass * L2 * c12 * (L2 * c12 + L1 * c1)
          + knee_mass * Lc2 * c12 * (Lc2 * c12 + L1 * c1);
    m13 = Iyy3;
    m21 = m12;
    m22 = wheel_mass * L2 * L2  + knee_mass * Lc2 * Lc2 + Iyy2 + Iyy3;
    m23 = Iyy3;
    m31 = m13;
    m32 = m23;
    m33 = Iyy3;
    M << m11, m12, m13,
          m21, m22, m23,
          m31, m32, m33;
    return M;

}
// 计算科氏力和向心力矩阵
template <typename T>
Mat3<T> QuadrupedRobot<T>::calcCoriolisForce(const Vec3<T> &theta,const Vec3<T> &dtheta){
    // std::cout<<"coriolisforce_1"<<std::endl;
    T c1 = std::cos(theta(0));
    T c2 = std::cos(theta(1));
    T s1 = std::sin(theta(0));
    T s2 = std::sin(theta(1));
    T C12 = c1*c2 - s1*s2;
    T s12 = s1*c2 + c1*s2;
    T dq1 = dtheta(0);
    T dq2 = dtheta(1);
    c11 = dq2 * (L2 * wheel_mass * C12 *(L2 *s12 + L1 * s1) 
            + Lc2 * knee_mass * C12 * (Lc2 * s12 + L1 * s1)
            -L2 * wheel_mass * s12 * (L2 * C12 + L1 * c1)
            -Lc2 * knee_mass * s12 * (Lc2 * C12 + L1 * c1));
    c12 = dq1 *(L2 * wheel_mass * C12 * (L2 * s12 + L1 * s1)
           + Lc2 * knee_mass * C12 * (Lc2 * s12 + L1 * s1)
           -L2 * wheel_mass * s12 * (L2 * C12 + L1 * c1)
            -Lc2 * knee_mass * s12 * (Lc2 * C12 + L1 * c1))
            + dq2 * (L2 * wheel_mass * C12 * (L2 * s12 + L1 * s1)
            + Lc2 * knee_mass * C12 * (Lc2 * s12 + L1 * s1)
            -L2 * wheel_mass * s12 * (L2 * C12 + L1 * c1)
            -Lc2 * knee_mass * s12 * (Lc2 * C12 + L1 * c1));
    c13 = c22 =c23 =c31 =c32 = c33 =0;
    c21 = -dq1 * (L2 * wheel_mass * C12 * (L2 * s12 + L1 * s1)
            + Lc2 * knee_mass * C12 * (Lc2 * s12 + L1 * s1)
            -L2 * wheel_mass * s12 * (L2 * C12 + L1 * c1)
            -Lc2 * knee_mass * s12 * (Lc2 * C12 + L1* c1));
    C << c11, c12, c13,
        c21, c22, c23,
        c31, c32, c33;
    return C;

}

//计算重力项矩阵
template <typename T>
Vec3<T> QuadrupedRobot<T>::calcGravityForce(const Vec3<T> &theta){
    // std::cout<<"gravity_1"<<std::endl;
    // std::cout<<"theta"<<theta<<std::endl;
    T c1 = std::cos(theta(0));
    T c2 = std::cos(theta(1));
    T s1 = std::sin(theta(0));
    T s2 = std::sin(theta(1));
    T c12 = c1*c2 - s1*s2;
    T s12 = s1*c2 + c1*s2;
    // std::cout<<"gravity_1"<<std::endl;
    // std::cout<<"g"<< _g <<std::endl;
    g1 = -_g * ( wheel_mass * (L2 * c12 + L1 * c1) 
            + knee_mass * (Lc2 * c12 + L1 * c1) 
            + hip_mass * (Lc1 * c1));
    g2 = -_g * (wheel_mass * (L2 * c12) 
            + knee_mass * Lc2 * c12 );
    g3 = 0.0;
    G << g1, g2, g3;
    // std::cout<<"gravity_2"<<std::endl;
    return G;
}

//计算关节力矩、无接触力的情况
template <typename T>
Vec3<T> QuadrupedRobot<T>::calcJointTorque(const Vec3<T> &theta,const Vec3<T> &dtheta,const Vec3<T> &ddtheta){
    Vec3<T> Torque;  //关节力矩
    Torque = calcInertialMatrix(theta) * ddtheta 
            + calcCoriolisForce(theta,dtheta) * dtheta 
            + calcGravityForce(theta);
    return Torque;
}

// 计算关节力矩，带接触力
template <typename T>
Vec3<T> QuadrupedRobot<T>::calcJointTorqueForce(const Vec3<T> &theta,const Vec3<T> &dtheta,const Vec3<T> &ddtheta,const Vec3<T> &F){
    Vec3<T> Torque;  //关节力矩
    // std::cout << "theta: " << theta << std::endl;
    Torque = calcInertialMatrix(theta) * ddtheta 
            + calcCoriolisForce(theta,dtheta) * dtheta 
            + calcGravityForce(theta) 
            + F;
    // std::cout << "不带接触力的Torque: " << calcInertialMatrix(theta) * ddtheta 
    // + calcCoriolisForce(theta,dtheta) * dtheta 
    // + calcGravityForce(theta) << std::endl;
    return Torque;
}
//无接触力计算关节驱动力矩
template <typename T>
Vec34<T> QuadrupedRobot<T>::calcJointTorqueTotal(const Vec34<T> &theta,const Vec34<T> &dtheta,const Vec34<T> &ddtheta){
    Vec34<T> _joint_torque; //所有的关节力矩
    for(int i = 0;i<4;i++)
    {
        _joint_torque.col(i) = calcJointTorque(theta.col(i),dtheta.col(i),ddtheta.col(i));
    }

    return _joint_torque;
}

//有接触力，计算关节力矩
template <typename T>
Vec34<T> QuadrupedRobot<T>::calcJointTorqueForceTotal(const Vec34<T> &theta,const Vec34<T> &dtheta,const Vec34<T> &ddtheta,const Vec34<T> &F){
    Vec34<T> _joint_torque; //所有的关节力矩
    Vec4<T> F_temp;
    // std::cout << "theta: " << theta<< std::endl;
    for(int i = 0;i<4;i++){
    // {   std::cout << "theta.col(i): " << theta.col(i) << std::endl;
        _joint_torque.col(i) = calcJointTorqueForce(theta.col(i),dtheta.col(i),ddtheta.col(i),F.col(i));
    }

    return _joint_torque;
}
template <typename T>
Vec34<T> QuadrupedRobot<T>::calcJointAcc(const Vec34<T> &theta,const Vec34<T> &dtheta,const Vec34<T> &ddtheta,const Vec34<T> &contact_torque,const Vec34<T> &_joint_torque){
    Vec34<T> _joint_acc; //所有的关节加速度
    for(int i = 0;i<4;i++){
        _joint_acc.col(i) = calcInertialMatrix(theta.col(i)).inverse() * (_joint_torque.col(i) 
        - contact_torque.col(i)
        - calcCoriolisForce(theta.col(i),dtheta.col(i)) * dtheta.col(i) 
        - calcGravityForce(theta.col(i)));
     }
     return _joint_acc;
}
// 计算每个刚体的位置、速度和加速度   这个函数已经修改正确了！不准再改！  模型也不许改了！！！
template <typename T>
void QuadrupedRobot<T>::forwardKinematics()     //所有关节的正向运动学计算
{
    if (_kinematicsUpToDate) return;
    
    // 基座姿态转换（世界坐标系到基座坐标系）

    // 浮动基座的变换矩阵，前三行是姿态，后三行是位置。左下是坐标系原点偏移r引起的线速度耦合项
    // _state.bodyPosition(2) = 0.15;
    _Xup[5] = createSXform(quaternionToRotationMatrix(_state.bodyOrientation),_state.bodyPosition); //计算浮动基的姿态和位置
    _v[5] = _state.bodyVelocity;    //机身角速度和线速度！！世界坐标系下的
    // std::cout << "_Xup" << _Xup[5] << std::endl;
    // std::cout << "pos_body" << _state.bodyPosition << std::endl;

    // std::cout << "pos_v" << _v[5] << std::endl;
    // // -2.09439, 1.347197, 0, -2.09439, 1.347197, 0, 
    // // //                         -2.09439, 1.347197, 0, -2.09439, 1.347197, 0
    // _state.q[0] = -2.09439;
    // _state.q[1] = 1.347197;
    // _state.q[2] = 0;
    // _state.q[3] = -2.09439;
    // _state.q[4] = 1.347197;
    // _state.q[5] = 0;
    // _state.q[6] = -2.09439;
    // _state.q[7] = 1.347197;
    // _state.q[8] = 0;
    // _state.q[9] = -2.09439;
    // _state.q[10] = 1.347197;
    // _state.q[11] = 0;
    for(size_t i = 6;i < _nDof ; ++i)      
    {
    // 固定关节类型为旋转关节，轴为 Y 轴
    //6*6的两个旋转矩阵
    _state.q[i-6] *= -1;
    _state.qd[i-6] *= -1;
    Mat6<T> XJ = jointXform(JointType::Revolute, CoordinateAxis::Y, _state.q[i - 6]);  //转动关节的旋转矩阵6*6 左上和右下，遍历所有的关节角度
    _Xup[i] = XJ * _Xtree[i];    //—Xtree 是一个空间变换矩阵6*6 [R 0 0 R] xtree没问题      空间变换矩阵，从父连杆到子连杆的    每个旋；量的变换矩阵
    // std::cout<<"Xtree,平移矩阵第"<<i<<"个"<<_Xtree[i]<<std::endl;    //这个是改对了
    _S[i] = jointMotionSubspace<T>(JointType::Revolute, CoordinateAxis::Y);   //[0 1 0 0 0 0]
    SVec<T> vJ = _S[i] * _state.qd[i - 6];      //6*1  * 1    转动速度 只有角速度
    _v[i] = _Xup[i] * _v[_parents[i]] + vJ;    //连杆的角速度和线速度6*1  父坐标系下
      // std::cout<<"parents"<<_parents[i]<<std::endl; 
    Mat6<T> XJrot = jointXform(JointType::Revolute, CoordinateAxis::Y,            //转子的旋转矩阵[R 0, 0 R]
                               _state.q[i - 6] * _gearRatios[i]);
    _Srot[i] = _S[i] * _gearRatios[i];             //[0 1 0 0 0 0]  绕Y轴旋转  
    SVec<T> vJrot = _Srot[i] * _state.qd[i - 6];          //转子空间绕y轴角速度（自己的坐标系下）
    // std::cout<<"xROT" << _Xrot[i] << std::endl;
    _Xuprot[i] = XJrot * _Xrot[i];              //_Xrot是电机转子的相对于关节坐标系的旋转矩阵 6*6
    _vrot[i] = _Xuprot[i] * _v[_parents[i]] + vJrot;      //转到父坐标系下求解

    _c[i] = motionCrossProduct(_v[i], vJ);           //计算连杆的运动叉积
    // std::cout<<"c" << _c[i] << std::endl;
    _crot[i] = motionCrossProduct(_vrot[i], vJrot);    
    // std::cout<<"crot" << _crot[i] << std::endl;
    // std::cout << "Joint " << i << " XJ:\n" << XJ << std::endl;       //所有关节的旋转矩阵
    // std::cout << "Joint " << i << " Xup:\n" << _Xup[i] << std::endl;
    // std::cout << "Joint " << i << " velocity (vJ): " << vJ.transpose() << std::endl;
    // std::cout << "Joint " << i << " parent velocity: " << _v[_parents[i]].transpose() << std::endl;
    // std::cout << "c[i]"  << i  << "个" << _c[i].transpose() << std::endl;

   
  }
    
    // 计算每个刚体的绝对变换矩阵，都是在父坐标系下！！
    for (size_t i = 5; i < _nDof; i++) {
        if (_parents[i] == 0) {
        _Xa[i] = _Xup[i];  // 浮动基座   6*6的矩阵
        } else {
        _Xa[i] = _Xup[i] * _Xa[_parents[i]];
        }
    }
    // 计算地面接触点的位置和速度 世界坐标系下  
    // std::cout << "Ground Contact Points:" << _nGroundContact << std::endl;  20 个  8+12
    //
    for (size_t j = 0; j < _nGroundContact; j++) {
        if (!_compute_contact_info[j]) continue;
        size_t i = _gcParent.at(j);
        // std::cout << "Contact " << j << " parent body Xa:\n" << _Xa[i] << std::endl;
        Mat6<T> Xai = invertSXform(_Xa[i]);     // 逆变换矩阵运动学

        SVec<T> vSpatial = Xai * _v[i];
        _pGC.at(j) = sXFormPoint(Xai, _gcLocation.at(j));    
        Vec3<T> gc_local = _gcLocation.at(j);  // 接触点在当前连杆坐标系下的位置
        // std::cout << "Local GC position: " << gc_local.transpose() << std::endl;
         // 输出接触点逆变换后的坐标
        // std::cout << "Contact " << j << " Xai (Inverse):\n" << Xai.template block<3,3>(0,0) << "\nPosition: " << Xai.template block<3,1>(3,0).transpose() << std::endl;
        _vGC.at(j) = spatialToLinearVelocity(vSpatial, _pGC.at(j));
        // // std::cout << "LF-LR-RR-RF" <<std::endl;
        // std::cout << "pGC: " << _pGC.at(j).transpose() << std::endl;       //12个接触点在世界坐标系下的坐标
        // std::cout << "vGC: " << _vGC.at(j).transpose() << std::endl;
    }
    _kinematicsUpToDate = true;

}



/*!
 * Add a ground contact point to a model
 * @param bodyID The ID of the body containing the contact point
 * @param location The location (in body coordinate) of the contact point
 * @param isFoot True if foot or not.
 * @return The ID of the ground contact point
 */
template <typename T>   //添加一个地面接触点，相当于是在初始化这个接触点的世界坐标下的位置、速度和父连杆的ID和在父连杆中的接触位置
int QuadrupedRobot<T>::addGroundContactPoint(int bodyID,
                                                const Vec3<T> &location,   //接触点在连杆坐标系中的位置
                                                bool isFoot) {
  if ((size_t)bodyID >= _nDof) {
    throw std::runtime_error(
        "addGroundContactPoint got invalid bodyID: " + std::to_string(bodyID) +
        " nDofs: " + std::to_string(_nDof) + "\n");
  }

  // std::cout << "pt-add: " << location.transpose() << "\n";
  _gcParent.push_back(bodyID);      //足端接触点的连杆ID 机身左上6
  _gcLocation.push_back(location);   //接触点的位置在连杆坐标系中的

  Vec3<T> zero3 = Vec3<T>::Zero();

  _pGC.push_back(zero3);               //接触点的全局位置坐标
  _vGC.push_back(zero3);              //接触点的全局速度

  D3Mat<T> J(3, _nDof);     //3*18     存储接触点的雅可比矩阵
  J.setZero();

  _Jc.push_back(J);                         //,有20个大小的_Jc 存储接触点的雅可比矩阵
  _Jcdqd.push_back(zero3);                     //存储雅可比时间导数与关节速度的乘积
  _compute_contact_info.push_back(false);      //是否计算接触点的雅可比矩阵，不是足端接触不计算

  // if(_nGroundContact > 7)
  // {_compute_contact_info.push_back(true);  }     //相当于在初始化它的大小

  // add foot to foot list
  if (isFoot) {
    _footIndicesGC.push_back(_nGroundContact);             //记录足端接触点的索引
    _compute_contact_info[_nGroundContact] = true;     //启用该接触点的力控计算
    // std::cout << "foot index: " << _nGroundContact << std::endl;      //10   13   16  19
  }
  
  resizeSystemMatricies();  //调整重力矩阵、科氏力矩阵、重力矩阵大小 
  // std::cout << "_ncgroundcontact" << _nGroundContact<< std::endl;  //最终是12个
  return _nGroundContact++;
}

/*!
 * Add the bounding points of a box to the contact model. Assumes the box is
 * centered around the origin of the body coordinate system and is axis aligned.
 * 将框的边界点添加到接触模型中。假设盒子是
 * 以身体坐标系统的原点为中心，并与轴线对齐。
 */
template <typename T>   //机身的ID都是5     //从LF -LR -RR -RF
void QuadrupedRobot<T>::addGroundContactBoxPoints(int bodyId,
                                                     const Vec3<T> &dims) {
   addGroundContactPoint(bodyId, Vec3<T>( dims(0),  dims(1),  dims(2))/2);     //5    
   addGroundContactPoint(bodyId, Vec3<T>(-dims(0),  dims(1),  dims(2))/2);    //5
   addGroundContactPoint(bodyId, Vec3<T>( -dims(0), -dims(1),  dims(2))/2);    //5
   addGroundContactPoint(bodyId, Vec3<T>(dims(0), -dims(1),  dims(2))/2);    //5

  //addGroundContactPoint(bodyId, Vec3<T>(dims(0), dims(1), 0.) / 2);
  //addGroundContactPoint(bodyId, Vec3<T>(-dims(0), dims(1), 0.) / 2);
  //addGroundContactPoint(bodyId, Vec3<T>(dims(0), -dims(1), 0.) / 2);
  //addGroundContactPoint(bodyId, Vec3<T>(-dims(0), -dims(1), 0.) / 2);

  addGroundContactPoint(bodyId, Vec3<T>(dims(0), dims(1), -dims(2)) / 2);    //5
  addGroundContactPoint(bodyId, Vec3<T>(-dims(0), dims(1), -dims(2)) / 2);   //5
  addGroundContactPoint(bodyId, Vec3<T>(-dims(0), -dims(1), -dims(2)) / 2);   //5
  addGroundContactPoint(bodyId, Vec3<T>(dims(0), -dims(1), -dims(2)) / 2);   //5
}

/*!
 * Create the floating body
 * @param inertia Spatial inertia of the floating body
 */
template <typename T>
void QuadrupedRobot<T>::addBase(const SpatialInertia<T> &inertia) {
  if (_nDof) {
    throw std::runtime_error("Cannot add base multiple times!\n");
  }

  Mat6<T> eye6 = Mat6<T>::Identity();
  Mat6<T> zero6 = Mat6<T>::Zero();
  SpatialInertia<T> zeroInertia(zero6);
  // the floating base has 6 DOFs
  _nDof = 6;
  for (size_t i = 0; i < 6; i++) {
    _parents.push_back(0);
    _gearRatios.push_back(0);
    _jointTypes.push_back(JointType::Nothing);  // doesn't actually matter
    _jointAxes.push_back(CoordinateAxis::X);    // doesn't actually matter
    _Xtree.push_back(eye6);
    _Ibody.push_back(zeroInertia);
    _Xrot.push_back(eye6);
    _Irot.push_back(zeroInertia);
    _bodyNames.push_back("N/A");
  }

  _jointTypes[5] = JointType::FloatingBase;
  _Ibody[5] = inertia;
  _gearRatios[5] = 1;
  _bodyNames[5] = "Floating Base";
  addDynamicsVars(6);
}

/*!
 * Create the floating body
 * @param mass Mass of the floating body
 * @param com  Center of mass of the floating body
 * @param I    Rotational inertia of the floating body
 */
template <typename T>
void QuadrupedRobot<T>::addBase(T mass, const Vec3<T> &com,
                                   const Mat3<T> &I) {
  SpatialInertia<T> IS(mass, com, I);
  addBase(IS);
}
/*!
 * Add a body
 * @param inertia The inertia of the body
 * @param rotorInertia The inertia of the rotor the body is connected to
 * @param gearRatio The gear ratio between the body and the rotor
 * @param parent The parent body, which is also assumed to be the body the rotor
 * is connected to
 * @param jointType The type of joint (prismatic or revolute)
 * @param jointAxis The joint axis (X,Y,Z), in the parent's frame
 * @param Xtree  The coordinate transformation from parent to this body
 * @param Xrot  The coordinate transformation from parent to this body's rotor
 * @return The body's ID (can be used as the parent)
 * 向动力学模型中添加新的刚体（连杆）及其相关信息
  */
  template <typename T>
  int QuadrupedRobot<T>::addBody(const SpatialInertia<T> &inertia,
                                    const SpatialInertia<T> &rotorInertia,
                                    T gearRatio, int parent, JointType jointType,
                                    CoordinateAxis jointAxis,
                                    const Mat6<T> &Xtree, const Mat6<T> &Xrot) {
    if ((size_t)parent >= _nDof) {
      throw std::runtime_error(
          "addBody got invalid parent: " + std::to_string(parent) +
          " nDofs: " + std::to_string(_nDof) + "\n");
    }

    _parents.push_back(parent);
    _gearRatios.push_back(gearRatio);
    _jointTypes.push_back(jointType);
    _jointAxes.push_back(jointAxis);
    _Xtree.push_back(Xtree);       //这是什么东西
    _Xrot.push_back(Xrot);
    _Ibody.push_back(inertia);
    _Irot.push_back(rotorInertia);
    _nDof++;

    addDynamicsVars(1);

    return _nDof;
  }

  /*!
  * Add a body
  * @param inertia The inertia of the body
  * @param rotorInertia The inertia of the rotor the body is connected to
  * @param gearRatio The gear ratio between the body and the rotor
  * @param parent The parent body, which is also assumed to be the body the rotor
  * is connected to
  * @param jointType The type of joint (prismatic or revolute)
  * @param jointAxis The joint axis (X,Y,Z), in the parent's frame
  * @param Xtree  The coordinate transformation from parent to this body
  * @param Xrot  The coordinate transformation from parent to this body's rotor
  * @return The body's ID (can be used as the parent)
  */
  template <typename T>
  int QuadrupedRobot<T>::addBody(const MassProperties<T> &inertia,
                                    const MassProperties<T> &rotorInertia,
                                    T gearRatio, int parent, JointType jointType,
                                    CoordinateAxis jointAxis,
                                    const Mat6<T> &Xtree, const Mat6<T> &Xrot) {
    return addBody(SpatialInertia<T>(inertia), SpatialInertia<T>(rotorInertia),
                  gearRatio, parent, jointType, jointAxis, Xtree, Xrot);
  }

template <typename T>
void QuadrupedRobot<T>::check() {
  if (_nDof != _parents.size())
    throw std::runtime_error("Invalid dof and parents length");
}

/*!
 * Compute the total mass of bodies which are not rotors.
 * @return
 */
template <typename T>
T QuadrupedRobot<T>::totalNonRotorMass() {          //计算非旋转器的总质量
  T totalMass = 0;
  for (size_t i = 0; i < _nDof; i++) {
    totalMass += _Ibody[i].getMass();
  }
  return totalMass;
}

/*!
 * Compute the total mass of bodies which are not rotors
 * @return
 */
template <typename T>
T QuadrupedRobot<T>::totalRotorMass() {              //计算旋转器的总质量
  T totalMass = 0;
  for (size_t i = 0; i < _nDof; i++) {
    totalMass += _Irot[i].getMass();
  }
  return totalMass;
}


template <typename T>
void QuadrupedRobot<T>::contactJacobians() {             //计算关节雅可比矩阵     
  forwardKinematics();                                      //更新关节状态
  biasAccelerations();                                   //偏值加速度，科里奥力力加速度

  for (size_t k = 0; k < _nGroundContact; k++) {                //n是地面接触点的数量
    if (!_compute_contact_info[k]) continue;             //如果接触点不参与计算，则跳过
    _Jc[k].setZero();                                 //接触雅可比矩阵
    _Jcdqd[k].setZero();                                 //偏置加速度矩阵，（接触点的非关节驱动加速度）

    // Skip it if we don't care about it
    size_t i = _gcParent.at(k);                           //每个关节点到父关节的索引 678910111213-19
    // std::cout << "第i个父关节的索引: " << i << std::endl;
    // Rotation to absolute coords
    Mat3<T> Rai = _Xa[i].template block<3, 3>(0, 0).transpose();    //12个接触点
    Mat6<T> Xc = createSXform(Rai, _gcLocation.at(k));              //生成从连杆到接触点的空间变换矩阵Xc,包括旋转和位置变换。这就是单位矩阵，刚好接触点就是坐标系的原点。
    // std::cout << "Xc: " << Xc << std::endl;
    // Bias acceleration
    SVec<T> ac = Xc * _avp[i];             
    SVec<T> vc = Xc * _v[i];        

    // Correct to classical
    _Jcdqd[k] = spatialToLinearAcceleration(ac, vc);             //将空间加速度转换为线加速度。得到接触点的偏置加速度。

    // rows for linear velcoity in the world      
    D3Mat<T> Xout = Xc.template bottomRows<3>();            // 获取Xc矩阵的下三行，即从Xc到接触点的空间变换矩阵。
    // std::cout << "Xout: " << Xout << std::endl;     // Xout是Xc矩阵的下三行，是机身的旋转矩阵
    // from tips to base
    while (i > 5) {
      _Jc[k].col(i) = Xout * _S[i];                         //将Xc矩阵的下三行乘以S[i]，得到每个接触点的雅可比矩阵。
      Xout = Xout * _Xup[i];
      i = _parents[i];                                       //每个接触点的父关节
    }
    _Jc[k].template leftCols<6>() = Xout;
    // std::cout << "Jc: " << k << "个点" <<std::endl << _Jc[k] << std::endl;
  }
}

/*!
 * (Support Function) Computes velocity product accelerations of
 * each link and rotor _avp, and _avprot
 */
template <typename T>
void QuadrupedRobot<T>::biasAccelerations() {      //这个是只算速度
  if (_biasAccelerationsUpToDate) return;
  forwardKinematics();                        //这里不一定会计算，
  // velocity product acceelration of base
  _avp[5] << 0, 0, 0, 0, 0, 0;            //基座的

  // from base to tips
  for (size_t i = 6; i < _nDof; i++) {
    // Outward kinamtic propagtion
    _avp[i] = _Xup[i] * _avp[_parents[i]] + _c[i];
    _avprot[i] = _Xuprot[i] * _avp[_parents[i]] + _crot[i];
  }
  _biasAccelerationsUpToDate = true;
}

/*!
 * Computes the generalized gravitational force (G) in the inverse dynamics
 * @return G (_nDof x 1 vector)
 */
template <typename T>
DVec<T> QuadrupedRobot<T>::generalizedGravityForce() {
  compositeInertias();
  SVec<T> aGravity;
  aGravity << 0, 0, 0, _gravity[0], _gravity[1], _gravity[2];
  _ag[5] = _Xup[5] * aGravity;

  // Gravity comp force is the same as force required to accelerate
  // oppostite gravity
  _G.template topRows<6>() = -_IC[5].getMatrix() * _ag[5];
  for (size_t i = 6; i < _nDof; i++) {
    _ag[i] = _Xup[i] * _ag[_parents[i]];
    _agrot[i] = _Xuprot[i] * _ag[_parents[i]];

    // body and rotor
    _G[i] = -_S[i].dot(_IC[i].getMatrix() * _ag[i]) -
            _Srot[i].dot(_Irot[i].getMatrix() * _agrot[i]);
  }
  return _G;
}

/*!
 * Computes the generalized coriolis forces (Cqd) in the inverse dynamics
 * @return Cqd (_nDof x 1 vector)
 */
template <typename T>
DVec<T> QuadrupedRobot<T>::generalizedCoriolisForce() {
  biasAccelerations();

  // Floating base force
  Mat6<T> Ifb = _Ibody[5].getMatrix();
  SVec<T> hfb = Ifb * _v[5];
  _fvp[5] = Ifb * _avp[5] + forceCrossProduct(_v[5], hfb);

  for (size_t i = 6; i < _nDof; i++) {
    // Force on body i
    Mat6<T> Ii = _Ibody[i].getMatrix();
    SVec<T> hi = Ii * _v[i];
    _fvp[i] = Ii * _avp[i] + forceCrossProduct(_v[i], hi);

    // Force on rotor i
    Mat6<T> Ir = _Irot[i].getMatrix();
    SVec<T> hr = Ir * _vrot[i];
    _fvprot[i] = Ir * _avprot[i] + forceCrossProduct(_vrot[i], hr);
  }

  for (size_t i = _nDof - 1; i > 5; i--) {
    // Extract force along the joints
    _Cqd[i] = _S[i].dot(_fvp[i]) + _Srot[i].dot(_fvprot[i]);

    // Propage force down the tree
    _fvp[_parents[i]] += _Xup[i].transpose() * _fvp[i];
    _fvp[_parents[i]] += _Xuprot[i].transpose() * _fvprot[i];
  }

  // Force on floating base
  _Cqd.template topRows<6>() = _fvp[5];
  return _Cqd;
}

template <typename T>
Mat3<T> QuadrupedRobot<T>::getOrientation(int link_idx) {
  forwardKinematics();
  Mat3<T> Rai = _Xa[link_idx].template block<3, 3>(0, 0);
  Rai.transposeInPlace();
  return Rai;
}


template <typename T>
Vec3<T> QuadrupedRobot<T>::getPosition(const int link_idx)
{
  forwardKinematics();
  Mat6<T> Xai = invertSXform(_Xa[link_idx]); // from link to absolute
  Vec3<T> link_pos = sXFormPoint(Xai, Vec3<T>::Zero());
  return link_pos;
}

template <typename T>
Vec3<T> QuadrupedRobot<T>::getPosition(const int link_idx, const Vec3<T> & local_pos)
{
  forwardKinematics();
  Mat6<T> Xai = invertSXform(_Xa[link_idx]); // from link to absolute
  Vec3<T> link_pos = sXFormPoint(Xai, local_pos);
  return link_pos;
}

template <typename T>
Vec3<T> QuadrupedRobot<T>::getLinearAcceleration(const int link_idx,
                                                    const Vec3<T> &point) {
  forwardAccelerationKinematics();
  Mat3<T> R = getOrientation(link_idx);
  return R * spatialToLinearAcceleration(_a[link_idx], _v[link_idx], point);
}

template <typename T>
Vec3<T> QuadrupedRobot<T>::getLinearAcceleration(const int link_idx) {
  forwardAccelerationKinematics();
  Mat3<T> R = getOrientation(link_idx);
  return R * spatialToLinearAcceleration(_a[link_idx], _v[link_idx], Vec3<T>::Zero());
}


template <typename T>
Vec3<T> QuadrupedRobot<T>::getLinearVelocity(const int link_idx,
                                                const Vec3<T> &point) {
  forwardKinematics();
  Mat3<T> Rai = getOrientation(link_idx);
  return Rai * spatialToLinearVelocity(_v[link_idx], point);
}

template <typename T>
Vec3<T> QuadrupedRobot<T>::getLinearVelocity(const int link_idx) {
  forwardKinematics();
  Mat3<T> Rai = getOrientation(link_idx);
  return Rai * spatialToLinearVelocity(_v[link_idx], Vec3<T>::Zero());
}



template <typename T>
Vec3<T> QuadrupedRobot<T>::getAngularVelocity(const int link_idx) {
  forwardKinematics();
  Mat3<T> Rai = getOrientation(link_idx);
  // Vec3<T> v3 =
  return Rai * _v[link_idx].template head<3>();
  
}

template <typename T>
Vec3<T> QuadrupedRobot<T>::getAngularAcceleration(const int link_idx) {
  forwardAccelerationKinematics();
  Mat3<T> Rai = getOrientation(link_idx);
  return Rai * _a[link_idx].template head<3>();
}

/*!
 * (Support Function) Computes the composite rigid body inertia
 * of each subtree _IC[i] contains body i, and the body/rotor
 * inertias of all successors of body i.
 * (key note: _IC[i] does not contain rotor i)
 */
template <typename T>
void QuadrupedRobot<T>::compositeInertias() {
  if (_compositeInertiasUpToDate) return;

  forwardKinematics();
  // initialize
  for (size_t i = 5; i < _nDof; i++) {
    _IC[i].setMatrix(_Ibody[i].getMatrix());
  }

  // backward loop
  for (size_t i = _nDof - 1; i > 5; i--) {
    // Propagate inertia down the tree
    _IC[_parents[i]].addMatrix(_Xup[i].transpose() * _IC[i].getMatrix() *
                               _Xup[i]);
    _IC[_parents[i]].addMatrix(_Xuprot[i].transpose() * _Irot[i].getMatrix() *
                               _Xuprot[i]);
  }
  _compositeInertiasUpToDate = true;
}

/*!
 * Computes the Mass Matrix (H) in the inverse dynamics formulation
 * @return H (_nDof x _nDof matrix)
 */
template <typename T>
DMat<T> QuadrupedRobot<T>::massMatrix() {
  compositeInertias();
  _H.setZero();

  // Top left corner is the locked inertia of the whole system
  _H.template topLeftCorner<6, 6>() = _IC[5].getMatrix();

  for (size_t j = 6; j < _nDof; j++) {
    // f = spatial force required for a unit qdd_j
    SVec<T> f = _IC[j].getMatrix() * _S[j];
    SVec<T> frot = _Irot[j].getMatrix() * _Srot[j];

    _H(j, j) = _S[j].dot(f) + _Srot[j].dot(frot);

    // Propagate down the tree
    f = _Xup[j].transpose() * f + _Xuprot[j].transpose() * frot;
    size_t i = _parents[j];
    while (i > 5) {
      // in here f is expressed in frame {i}
      _H(i, j) = _S[i].dot(f);
      _H(j, i) = _H(i, j);

      // Propagate down the tree
      f = _Xup[i].transpose() * f;
      i = _parents[i];
    }

    // Force on floating base
    _H.template block<6, 1>(0, j) = f;
    _H.template block<1, 6>(j, 0) = f.adjoint();
  }
  return _H;
}

template <typename T>
void QuadrupedRobot<T>::forwardAccelerationKinematics() {
  if (_accelerationsUpToDate) {
    return;
  }

  forwardKinematics();
  biasAccelerations();

  // Initialize gravity with model info
  SVec<T> aGravity = SVec<T>::Zero();
  aGravity.template tail<3>() = _gravity;

  // Spatial force for floating base
  _a[5] = -_Xup[5] * aGravity + _dState.dBodyVelocity;

  // loop through joints
  for (size_t i = 6; i < _nDof; i++) {
    // spatial acceleration
    _a[i] = _Xup[i] * _a[_parents[i]] + _S[i] * _dState.qdd[i - 6] + _c[i];
    _arot[i] =
        _Xuprot[i] * _a[_parents[i]] + _Srot[i] * _dState.qdd[i - 6] + _crot[i];
  }
  _accelerationsUpToDate = true;
}

/*!
 * Computes the inverse dynamics of the system
 * @return an _nDof x 1 vector. The first six entries
 * give the external wrengh on the base, with the remaining giving the
 * joint torques
 */
template <typename T>
DVec<T> QuadrupedRobot<T>::inverseDynamics(
    const FBModelStateDerivative<T> &dState) {
  setDState(dState);
  forwardAccelerationKinematics();

  // Spatial force for floating base
  SVec<T> hb = _Ibody[5].getMatrix() * _v[5];
  _f[5] = _Ibody[5].getMatrix() * _a[5] + forceCrossProduct(_v[5], hb);

  // loop through joints
  for (size_t i = 6; i < _nDof; i++) {
    // spatial momentum
    SVec<T> hi = _Ibody[i].getMatrix() * _v[i];
    SVec<T> hr = _Irot[i].getMatrix() * _vrot[i];

    // spatial force
    _f[i] = _Ibody[i].getMatrix() * _a[i] + forceCrossProduct(_v[i], hi);
    _frot[i] =
        _Irot[i].getMatrix() * _arot[i] + forceCrossProduct(_vrot[i], hr);
  }

  DVec<T> genForce(_nDof);
  for (size_t i = _nDof - 1; i > 5; i--) {
    // Pull off compoents of force along the joint
    genForce[i] = _S[i].dot(_f[i]) + _Srot[i].dot(_frot[i]);

    // Propagate down the tree
    _f[_parents[i]] += _Xup[i].transpose() * _f[i];
    _f[_parents[i]] += _Xuprot[i].transpose() * _frot[i];
  }
  genForce.template head<6>() = _f[5];
  return genForce;
}

template <typename T>  //调整系统矩阵的大小，
void QuadrupedRobot<T>::resizeSystemMatricies() {
  _H.setZero(_nDof, _nDof);      //质量矩阵 18*18
  _C.setZero(_nDof, _nDof);      // Coriolis矩阵
  _Cqd.setZero(_nDof);                // Coriolis矩阵
  _G.setZero(_nDof);                  // Gravity矩阵
  for (size_t i = 0; i < _J.size(); i++) {
    _J[i].setZero(6, _nDof);
    _Jdqd[i].setZero();
  }

  for (size_t i = 0; i < _Jc.size(); i++) {
    _Jc[i].setZero(3, _nDof);
    _Jcdqd[i].setZero();
  }
  _qdd_from_subqdd.resize(_nDof - 6, _nDof - 6);    //12*12
  _qdd_from_base_accel.resize(_nDof - 6, 6);         //12*6
  _state.q = DVec<T>::Zero(_nDof - 6);
  _state.qd = DVec<T>::Zero(_nDof - 6);
}
template <typename T>
void QuadrupedRobot<T>::runABA(const DVec<T> &tau,
                                  FBModelStateDerivative<T> &dstate) {
  (void)tau;
  forwardKinematics();
  updateArticulatedBodies();

  // create spatial vector for gravity
  SVec<T> aGravity;
  aGravity << 0, 0, 0, _gravity[0], _gravity[1], _gravity[2];

  // float-base articulated inertia
  SVec<T> ivProduct = _Ibody[5].getMatrix() * _v[5];
  _pA[5] = forceCrossProduct(_v[5], ivProduct);

  // loop 1, down the tree
  for (size_t i = 6; i < _nDof; i++) {
    ivProduct = _Ibody[i].getMatrix() * _v[i];
    _pA[i] = forceCrossProduct(_v[i], ivProduct);
   // same for rotors
    SVec<T> vJrot = _Srot[i] * _state.qd[i - 6];
    _vrot[i] = _Xuprot[i] * _v[_parents[i]] + vJrot;
    _crot[i] = motionCrossProduct(_vrot[i], vJrot);
    ivProduct = _Irot[i].getMatrix() * _vrot[i];
    _pArot[i] = forceCrossProduct(_vrot[i], ivProduct);
    }

  // adjust pA for external forces
  for (size_t i = 5; i < _nDof; i++) {
    // TODO add if statement (avoid these calculations if the force is zero)
    Mat3<T> R = rotationFromSXform(_Xa[i]);
    Vec3<T> p = translationFromSXform(_Xa[i]);
    Mat6<T> iX = createSXform(R.transpose(), -R * p);
    _pA[i] = _pA[i] - iX.transpose() * _externalForces.at(i);
  }

  // Pat's magic principle of least constraint
  for (size_t i = _nDof - 1; i >= 6; i--) {
    _u[i] = tau[i - 6] - _S[i].transpose() * _pA[i] -
            _Srot[i].transpose() * _pArot[i] - _U[i].transpose() * _c[i] -
            _Urot[i].transpose() * _crot[i];

    // articulated inertia recursion
    SVec<T> pa =
        _Xup[i].transpose() * (_pA[i] + _IA[i] * _c[i]) +
        _Xuprot[i].transpose() * (_pArot[i] + _Irot[i].getMatrix() * _crot[i]) +
        _Utot[i] * _u[i] / _d[i];
        _pA[_parents[i]] += pa;
  }

  // include gravity and compute acceleration of floating base
  SVec<T> a0 = -aGravity;
  SVec<T> ub = -_pA[5];
  _a[5] = _Xup[5] * a0;
  SVec<T> afb = _invIA5.solve(ub - _IA[5].transpose() * _a[5]);
  _a[5] += afb;

  // joint accelerations
  dstate.qdd = DVec<T>(_nDof - 6);
  for (size_t i = 6; i < _nDof; i++) {
    dstate.qdd[i - 6] =
        (_u[i] - _Utot[i].transpose() * _a[_parents[i]]) / _d[i];
    _a[i] = _Xup[i] * _a[_parents[i]] + _S[i] * dstate.qdd[i - 6] + _c[i];
  }

  // output
  RotMat<T> Rup = rotationFromSXform(_Xup[5]);
  dstate.dBodyPosition =
      Rup.transpose() * _state.bodyVelocity.template block<3, 1>(3, 0);
  dstate.dBodyVelocity = afb;
  // qdd is set in the for loop above
}
/*
用于更新关节加速度对系统动力学的影响
 */
template <typename T>
void QuadrupedRobot<T>::updateQddEffects() {
  if (_qddEffectsUpToDate) return;         
  updateForcePropagators();                   //更新力传播器，确保力传播相关的数据结构是最新的
  _qdd_from_base_accel.setZero();                   //存储基座加速度对系统状态导数的影响，设置为0矩阵
  _qdd_from_subqdd.setZero();                      //存储关节加速度对状态导数的影响，设置为0矩阵

  // Pass for force props
  // This loop is semi-equivalent to a cholesky factorization on H
  // akin to Featherstone's sparse operational space algo
  // These computations are for treating the joint rates like a task space
  // To do so, F computes the dynamic effect of torues onto bodies down the tree
  //
  for (size_t i = 6; i < _nDof; i++) {               //遍历所有的关节
    _qdd_from_subqdd(i - 6, i - 6) = 1;               //设置关节i自身的加速度对自身的影响力为1。
    SVec<T> F = (_ChiUp[i].transpose() - _Xup[i].transpose()) * _S[i];    //计算力向量F   
    size_t j = _parents[i];                                       //父连杆id
    while (j > 5) {                                        //向上遍历父连杆
      _qdd_from_subqdd(i - 6, j - 6) = _S[j].dot(F);
      F = _ChiUp[j].transpose() * F;                             //将力向量通过力传播矩阵传播到父关节j
      j = _parents[j];
    } 
    _qdd_from_base_accel.row(i - 6) = F.transpose();            //更新基座加速度的影响
  }
  _qddEffectsUpToDate = true;
}
/*
* Support function for contact inertia algorithms
* Comptues force propagators across each joint
* 支持功能的接触惯性算法
* 计算跨每个关节的力传播器
*/
template <typename T>
void QuadrupedRobot<T>::updateForcePropagators() {                    //力传播的数据结构
 if (_forcePropagatorsUpToDate) return;
 updateArticulatedBodies(); 
 for (size_t i = 6; i < _nDof; i++) {
   _ChiUp[i] = _Xup[i] - _S[i] * _Utot[i].transpose() / _d[i];                  //计算和更新力传播矩阵_chiup
 }
 _forcePropagatorsUpToDate = true;
}
/*!
 * Support function for the ABA
 * ABA的核心步骤，递归计算每个关节及其子树的组合惯性。为后续的力传播和动力学计算提供必要的数据。
 */
template <typename T>
void QuadrupedRobot<T>::updateArticulatedBodies() {
  if (_articulatedBodiesUpToDate) return;       //如果已经更新过，则直接返回

  forwardKinematics();                          //计算正向运动学，包括所有连杆的位资、速度和加速度

  _IA[5] = _Ibody[5].getMatrix();                         //初始化基座惯性矩阵

  // loop 1, down the tree
  for (size_t i = 6; i < _nDof; i++) {                              //向下遍历所有的关节
    _IA[i] = _Ibody[i].getMatrix();  // initialize 初始化自身的连杆惯性矩阵
    Mat6<T> XJrot = jointXform(_jointTypes[i], _jointAxes[i],            //计算关节的空间变换矩阵，基于关节类型，轴和角度
                               _state.q[i - 6] * _gearRatios[i]);
    _Xuprot[i] = XJrot * _Xrot[i];                      //结合转子惯性，计算关节旋转后的变换矩阵
    _Srot[i] = _S[i] * _gearRatios[i];                               //关节旋转后的运动子空间向量，考虑传动比
  }

  // Pat's magic principle of least constraint (Guass too!)
  for (size_t i = _nDof - 1; i >= 6; i--) {                            //向上遍历树结构
    _U[i] = _IA[i] * _S[i];                                      
    _Urot[i] = _Irot[i].getMatrix() * _Srot[i];                    //转子惯性与旋转后的运动子空间向量乘积
    _Utot[i] = _Xup[i].transpose() * _U[i] + _Xuprot[i].transpose() * _Urot[i];

    _d[i] = _Srot[i].transpose() * _Urot[i];
    _d[i] += _S[i].transpose() * _U[i];

    // articulated inertia recursion
    Mat6<T> Ia = _Xup[i].transpose() * _IA[i] * _Xup[i] +
                 _Xuprot[i].transpose() * _Irot[i].getMatrix() * _Xuprot[i] -
                 _Utot[i] * _Utot[i].transpose() / _d[i];
    _IA[_parents[i]] += Ia;
  }

  _invIA5.compute(_IA[5]);
  _articulatedBodiesUpToDate = true;
}

/*
* Populate member variables when bodies are added
在添加主体时填充成员变量
* @param count (6 for fb, 1 for joint)
*/
template <typename T>
void QuadrupedRobot<T>::addDynamicsVars(int count) {
if (count != 1 && count != 6) {
 throw std::runtime_error(
     "addDynamicsVars must be called with count=1 (joint) or count=6 "
     "(base).\n");    //添加新的自由度
    }

    Mat6<T> eye6 = Mat6<T>::Identity();         
    SVec<T> zero6 = SVec<T>::Zero();
    Mat6<T> zero66 = Mat6<T>::Zero();

    SpatialInertia<T> zeroInertia(zero66);             //初始化一个质量为0的刚体
    for (int i = 0; i < count; i++) {         //循环添加动力学变量
    _v.push_back(zero6);                              //线速度向量
    _vrot.push_back(zero6);                            //转子线速度向量
    _a.push_back(zero6);                                    //线加速度向量
    _arot.push_back(zero6);                            //角加速度向量
    _avp.push_back(zero6);              //广义加速度向量
    _avprot.push_back(zero6);
    _c.push_back(zero6);                //科氏力和向心力
    _crot.push_back(zero6);
    _S.push_back(zero6);               //描述关节自由度的空间向量
    _Srot.push_back(zero6);
    _f.push_back(zero6);                //作用在连杆上的力
    _frot.push_back(zero6);            //旋转分量的空间力
    _fvp.push_back(zero6);            //广义力，用于虚空原理计算
    _fvprot.push_back(zero6);       //广义旋转力
    _ag.push_back(zero6);            //重力引起的加速度
    _agrot.push_back(zero6);        //旋转分量的重力引起的加速度
    _IC.push_back(zeroInertia);              //连杆的空间惯性矩阵
    _Xup.push_back(eye6);                           //空间变换矩阵
    _Xuprot.push_back(eye6);
    _Xa.push_back(eye6);                         //一直到父坐标系的变换矩阵

    _ChiUp.push_back(eye6);            //子连杆到父连杆的运动传递矩阵
    _d.push_back(0.);                 //关节的阻尼系数
    _u.push_back(0.);                  //关节力矩输入
    _IA.push_back(eye6);              //连杆的质量和空间惯性矩阵

    _U.push_back(zero6);              //广义力向量
    _Urot.push_back(zero6);                //转子的广义力
    _Utot.push_back(zero6);             //综合外力，接触力、重力和惯性力
    _pA.push_back(zero6);                  //动量向量
    _pArot.push_back(zero6);              //旋转分量的动量
    _externalForces.push_back(zero6);    //外部作用力，接触力、环境干扰等
    }

    _J.push_back(D6Mat<T>::Zero(6, _nDof));     //添加雅可比矩阵和相关变量
    _Jdqd.push_back(SVec<T>::Zero());

    resizeSystemMatricies();
}
// /*
// 用于更新关节加速度对系统动力学的影响
//  */
// template <typename T>
// void QuadrupedRobot<T>::updateQddEffects() {
//   if (_qddEffectsUpToDate) return;         
//     updateForcePropagators();                   //更新力传播器，确保力传播相关的数据结构是最新的
//     _qdd_from_base_accel.setZero();                   //存储基座加速度对系统状态导数的影响，设置为0矩阵
//     _qdd_from_subqdd.setZero();                      //存储关节加速度对状态导数的影响，设置为0矩阵

//   // Pass for force props
//   // This loop is semi-equivalent to a cholesky factorization on H
//   // akin to Featherstone's sparse operational space algo
//   // These computations are for treating the joint rates like a task space
//   // To do so, F computes the dynamic effect of torues onto bodies down the tree
//   //
//   for (size_t i = 6; i < _nDof; i++) {               //遍历所有的关节
//     _qdd_from_subqdd(i - 6, i - 6) = 1;               //设置关节i自身的加速度对自身的影响力为1。
//     SVec<T> F = (_ChiUp[i].transpose() - _Xup[i].transpose()) * _S[i];    //计算力向量F   
//     size_t j = _parents[i];                                       //父连杆id
//     while (j > 5) {                                        //向上遍历父连杆
//       _qdd_from_subqdd(i - 6, j - 6) = _S[j].dot(F);
//       F = _ChiUp[j].transpose() * F;                             //将力向量通过力传播矩阵传播到父关节j
//       j = _parents[j];
//     } 
//     _qdd_from_base_accel.row(i - 6) = F.transpose();            //更新基座加速度的影响
//   }
//   _qddEffectsUpToDate = true;
// }

/*!
 * Apply a unit test force at a contact. Returns the inv contact inertia  in
 * that direction and computes the resultant qdd
 * @param gc_index index of the contact
 * @param force_ics_at_contact unit test forcoe
 * @params dstate - Output paramter of resulting accelerations
 * @return the 1x1 inverse contact inertia J H^{-1} J^T
 */
template <typename T>
T QuadrupedRobot<T>::applyTestForce(const int gc_index,
                                       const Vec3<T> &force_ics_at_contact,
                                       FBModelStateDerivative<T> &dstate_out) {
  forwardKinematics();
  updateArticulatedBodies();
  updateForcePropagators();
  updateQddEffects();

  size_t i_opsp = _gcParent.at(gc_index);
  size_t i = i_opsp;

  dstate_out.qdd.setZero();

  // Rotation to absolute coords
  Mat3<T> Rai = _Xa[i].template block<3, 3>(0, 0).transpose();
  Mat6<T> Xc = createSXform(Rai, _gcLocation.at(gc_index));

  // D is one column of an extended force propagator matrix (See Wensing, 2012
  // ICRA)
  SVec<T> F = Xc.transpose().template rightCols<3>() * force_ics_at_contact;

  double LambdaInv = 0;
  double tmp = 0;

  // from tips to base
  while (i > 5) {
    tmp = F.dot(_S[i]);
    LambdaInv += tmp * tmp / _d[i];
    dstate_out.qdd += _qdd_from_subqdd.col(i - 6) * tmp / _d[i];

    // Apply force propagator (see Pat's ICRA 2012 paper)
    // essentially, since the joint is articulated, only a portion of the force
    // is felt on the predecessor. So, while Xup^T sends a force backwards as if
    // the joint was locked, ChiUp^T sends the force backward as if the joint
    // were free
    F = _ChiUp[i].transpose() * F;
    i = _parents[i];
  }
  // TODO: Only carry out the QR once within update Aritculated Bodies
  dstate_out.dBodyVelocity = _invIA5.solve(F);
  LambdaInv += F.dot(dstate_out.dBodyVelocity);
  dstate_out.qdd += _qdd_from_base_accel * dstate_out.dBodyVelocity;

  return LambdaInv;
}

/*基于空间刚体动力学和Articulated Body Algorithm（ABA），
旨在高效地计算多体系统中施加力的动态影响。
通过空间向量的运算和力传播矩阵的应用，方法能够快速、准确地评估施加力对整个系统的影响，包括基座和各关节的加速度
*/
template <typename T>              
T QuadrupedRobot<T>::applyTestForce(const int gc_index,                                //接触点的索引
                                       const Vec3<T> &force_ics_at_contact,               //施加在接触点上的力
                                       DVec<T> &dstate_out) {                             //dstate_out是输出参数，结果加速度
  forwardKinematics();                                  //基于当前状态更新所有连杆的位姿、速度和加速度
  updateArticulatedBodies();        //更新系统中各连杆的组合惯性和其他动力学相关信息，涉及刚体动力学的计算
  updateForcePropagators();           //更新力传播矩阵，用于将某连杆上的力传递其父连杆
  updateQddEffects();                //更新关节加速度对系统动力学的影响
 
  size_t i_opsp = _gcParent.at(gc_index);              //i_opsp是接触点的父节点
  size_t i = i_opsp;                                     //当前接触点的父连杆索引

  dstate_out = DVec<T>::Zero(_nDof);                        //初始化状态导数输出

  // Rotation to absolute coords
  Mat3<T> Rai = _Xa[i].template block<3, 3>(0, 0).transpose();       //接触点坐标系旋转到绝对坐标
  Mat6<T> Xc = createSXform(Rai, _gcLocation.at(gc_index));          //创新一个空间变换矩阵，用于将接触点的力从接触坐标转换到世界坐标系

  // D is one column of an extended force propagator matrix (See Wensing, 2012
  // ICRA)
  SVec<T> F = Xc.transpose().template rightCols<3>() * force_ics_at_contact;        //计算空间力向量，提取变换矩阵的右半部分。F表示在世界坐标系中的力和力矩。

  T LambdaInv = 0;            //累积的逆惯性量度，最终返回值
  T tmp = 0;             

  // from tips to base                            //从接触点向基座传播力，并累积影响
  while (i > 5) {
    tmp = F.dot(_S[i]);                     //关节运动子空间向量，计算关节力 F*L
    LambdaInv += tmp * tmp / _d[i];              //d[i]关节的有效惯性量度。累积逆惯性贡献，基于施加力在关节方向上的分量和关节惯性。
    dstate_out.tail(_nDof - 6) += _qdd_from_subqdd.col(i - 6) * tmp / _d[i];      //更新关节加速度对状态导数的影响矩阵，

    // Apply force propagator (see Pat's ICRA 2012 paper)
    // essentially, since the joint is articulated, only a portion of the force
    // is felt on the predecessor. So, while Xup^T sends a force backwards as if
    // the joint was locked, ChiUp^T sends the force backward as if the joint
    // were free
    F = _ChiUp[i].transpose() * F;      //力传播矩阵，考虑关节是否自由或者锁定
    i = _parents[i];                    //向上遍历连杆链，继续传播力
  }

  dstate_out.head(6) = _invIA5.solve(F);                   //计算基座的线性和角加速度，基于施加的力         invIA5表示浮动基座的逆组合惯性矩阵
  LambdaInv += F.dot(dstate_out.head(6));                  //进一步累积逆惯性的量度
  dstate_out.tail(_nDof - 6) += _qdd_from_base_accel * dstate_out.head(6);               //基座加速度对关节加速度的影响矩阵*基座的线性和角加速度
 //更新关节的状态导数
  return LambdaInv;                                    //反映了施加力F对系统整体惯性的影响。这个标量值在动力学分析和控制中可能用于评估力的有效性或系统对力的响应能力
}
/*!
 * Compute the inverse of the contact inertia matrix (mxm)
 * @param force_ics_at_contact (3x1)
 *        e.g. if you want the cartesian inv. contact inertia in the z_ics
 *             force_ics_at_contact = [0 0 1]^T
 * @return the 1x1 inverse contact inertia J H^{-1} J^T
 */
template <typename T>
T QuadrupedRobot<T>::invContactInertia(const int gc_index,
                                          const Vec3<T> &force_ics_at_contact) {
  forwardKinematics();
  updateArticulatedBodies();
  updateForcePropagators();

  size_t i_opsp = _gcParent.at(gc_index);
  size_t i = i_opsp;

  // Rotation to absolute coords
  Mat3<T> Rai = _Xa[i].template block<3, 3>(0, 0).transpose();
  Mat6<T> Xc = createSXform(Rai, _gcLocation.at(gc_index));

  // D is one column of an extended force propagator matrix (See Wensing, 2012
  // ICRA)
  SVec<T> F = Xc.transpose().template rightCols<3>() * force_ics_at_contact;

  double LambdaInv = 0;
  double tmp = 0;

  // from tips to base
  while (i > 5) {
    tmp = F.dot(_S[i]);
    LambdaInv += tmp * tmp / _d[i];

    // Apply force propagator (see Pat's ICRA 2012 paper)
    // essentially, since the joint is articulated, only a portion of the force
    // is felt on the predecessor. So, while Xup^T sends a force backwards as if
    // the joint was locked, ChiUp^T sends the force backward as if the joint
    // were free
    F = _ChiUp[i].transpose() * F;
    i = _parents[i];
  }
  LambdaInv += F.dot(_invIA5.solve(F));
  return LambdaInv;
}

/*!
 * Compute the inverse of the contact inertia matrix (mxm)
 * @param force_directions (6xm) each column denotes a direction of interest
 *        col = [ moment in i.c.s., force in i.c.s.]
 *        e.g. if you want the cartesian inv. contact inertia
 *             force_directions = [ 0_{3x3} I_{3x3}]^T
 *             if you only want the cartesian inv. contact inertia in one
 * direction then use the overloaded version.
 * @return the mxm inverse contact inertia J H^{-1} J^T
 */
template <typename T>
DMat<T> QuadrupedRobot<T>::invContactInertia(
    const int gc_index, const D6Mat<T> &force_directions) {
  forwardKinematics();
  updateArticulatedBodies();
  updateForcePropagators();

  size_t i_opsp = _gcParent.at(gc_index);
  size_t i = i_opsp;

  // Rotation to absolute coords
  Mat3<T> Rai = _Xa[i].template block<3, 3>(0, 0).transpose();
  Mat6<T> Xc = createSXform(Rai, _gcLocation.at(gc_index));

  // D is a subslice of an extended force propagator matrix (See Wensing, 2012
  // ICRA)
  D6Mat<T> D = Xc.transpose() * force_directions;

  size_t m = force_directions.cols();

  DMat<T> LambdaInv = DMat<T>::Zero(m, m);
  DVec<T> tmp = DVec<T>::Zero(m);

  // from tips to base
  while (i > 5) {
    tmp = D.transpose() * _S[i];
    LambdaInv += tmp * tmp.transpose() / _d[i];

    // Apply force propagator (see Pat's ICRA 2012 paper)
    // essentially, since the joint is articulated, only a portion of the force
    // is felt on the predecessor. So, while Xup^T sends a force backwards as if
    // the joint was locked, ChiUp^T sends the force backward as if the joint
    // were free
    D = _ChiUp[i].transpose() * D;
    i = _parents[i];
  }

  // TODO: Only carry out the QR once within update Aritculated Bodies
  LambdaInv += D.transpose() * _invIA5.solve(D);

  return LambdaInv;
}
template class QuadrupedRobot<double>;
template class QuadrupedRobot<float>;

template <typename T>
xingtianRobot<T>::xingtianRobot(){
    this->_Legs[0] = new xingtianLeg<T>(0, Vec3<T>(0.19357, 0.2005, -0.03591));  //pH2B   //LF   X Y Z m
    this->_Legs[1] = new xingtianLeg<T>(1, Vec3<T>(-0.19357, 0.2005, -0.03591));           //LR
    this->_Legs[2] = new xingtianLeg<T>(2, Vec3<T>(-0.19357, -0.2005, -0.03591));           //RR
    this->_Legs[3] = new xingtianLeg<T>(3, Vec3<T>(0.19357, -0.2005, -0.03591));           //RF
    // 各个足端中性落脚点在机身坐标系下的坐标
    this->_feetPosNormalStand <<  0.24,  -0.24, -0.24, 0.24,
                           0.201,    0.201,  -0.201,  -0.201,
                           -0.250,     -0.250,  -0.250, -0.250;    // X Y Z
    this->_gravity << 0, 0, -9.81;
    this->_robVelLimitX << -0.10, 0.10;
    this->_robVelLimitY << -0.050, 0.050;
    this->_robVelLimitYaw << -0.80, 0.80;

    this->_mass = 41.56;    //kg
    this->_pcb << -0.000473, 0.0, -0.00063;     //重心位置
    // _Ib = Vec3<T>(0.0792, 0.2085, 0.2265).asDiagonal(); //简化模型的转动惯量
    this->_Ib << 0.333616693,0.000278946,0.000366942,              //机身在机身坐标系下的转动惯量
            0.000278946,1.404375499,-0.000151531,
            0.000366942,-0.000151531,1.555374018;   //m2
    //动力学模型
     //大腿、小腿、车轮转动惯量 ，在这里进行赋值！！
    this->_g = -9.81;
    this->Iyy1 = 0.001291752;
    this->Iyy2 = 0.000890565;
    this->Iyy3 = 0.013976692;
    this->hip_mass = 0.25201;    //大腿质量
    this->knee_mass = 0.102;    //小腿质量
    this->wheel_mass = 3.509;    //车轮质量
    this->L1 = 0.14; 
    this->L2 = 0.14;
    this->Lc1 = 0.0489;
    this->Lc2 = 0.08036;  //连杆长度以及重心位置  这些在这里进行赋值！！
}
template class xingtianRobot<double>;
template class xingtianRobot<float>;