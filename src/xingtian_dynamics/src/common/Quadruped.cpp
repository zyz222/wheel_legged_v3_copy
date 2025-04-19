/*! @file Quadruped.cpp
 *  @brief Data structure containing parameters for quadruped robot
 *
 *  This file contains the Quadruped class.  This stores all the parameters for
 * a quadruped robot.  There are utility functions to generate Quadruped objects
 * for xingtian 3 (and eventually mini-xingtian). There is a buildModel() method
 * which can be used to create a floating-base dynamics model of the quadruped.
 */

#include "common/Quadruped.h"
#include "common/spatial.h"
#include "common/orientation_tools.h"

using namespace ori;
using namespace spatial;

/*!
 * Build a QuadrupedRobot of the quadruped
 */
template <typename T>
bool Quadruped<T>::buildModel(QuadrupedRobot<T>& model) {
  // we assume the xingtian's body (not including rotors) can be modeled as a
  // uniformly distributed box.
  Vec3<T> bodyDims(_bodyLength, _bodyWidth, _bodyHeight);  //机身的长宽高属性
  // model.addBase(_bodyMass, Vec3<T>(0,0,0), rotInertiaOfBox(_bodyMass,
  // bodyDims));
  model.addBase(_bodyInertia); //创建浮动基座模型！
  // add contact for the xingtian's body
  model.addGroundContactBoxPoints(5, bodyDims);    //为机身添加接触模型，长方体的8个顶点 接触点ID是7

  const int baseID = 5;
  int bodyID = baseID;     //机身ID为5，开始就是5
  
  int sideSign = 0;      //左边为正，先控制左边！！！

  Mat3<T> I3 = Mat3<T>::Identity();

  // loop over 4 legs    
  for (int legID = 0; legID < 4; legID++) {
 
    bodyID++;
    // 关节坐标系的空间变换矩阵髋关节坐标系，以左前脚为准，为正
    // 这里需要对后腿的旋转矩阵进行修改！！！！
    Mat6<T> xtreeHip = createSXform(I3, withLegSignsHip<T>(_hipLocation, legID));    //空间变换矩阵 6*6   髋关节的旋转矩阵
    Mat6<T> xtreeHipRotor =createSXform(I3, withLegSignsHip<T>(_hipRotorLocation, legID));    //髋关节电机的旋转矩阵
    if (sideSign > 1) {           
      model.addBody(_hipInertia.flipAlongAxis(CoordinateAxis::Y),    //关于某个轴翻转转动惯量的
                    _hipRotorInertia.flipAlongAxis(CoordinateAxis::Y),
                    _hipGearRatio, baseID, JointType::Revolute,                 //baseID指的是父连杆的ID
                    CoordinateAxis::Y, xtreeHip, xtreeHipRotor);
    } else {
      model.addBody(_hipInertia, _hipRotorInertia, _hipGearRatio, baseID,   
                    JointType::Revolute, CoordinateAxis::Y, xtreeHip,
                    xtreeHipRotor);
    }
    // model.addGroundContactPoint(bodyID, Vec3<T>(_hipLinkLength, 0, 0), true);    
    model.addGroundContactPoint(bodyID, Vec3<T>(0, 0, 0), true);    
    // knee Joint
    bodyID++;
    Mat6<T> xtreeKnee = createSXform(I3,withLegSigns<T>(_kneeLocation, legID));
    Mat6<T> xtreeKneeRotor = createSXform(I3,withLegSigns<T>(_kneeRotorLocation, legID));
    if (sideSign > 1) {
      model.addBody(_kneeInertia.flipAlongAxis(CoordinateAxis::Y),
                    _kneeRotorInertia.flipAlongAxis(CoordinateAxis::Y),
                    _kneeGearRatio, bodyID - 1, JointType::Revolute,
                    CoordinateAxis::Y, xtreeKnee, xtreeKneeRotor);
      model.addGroundContactPoint(bodyID, Vec3<T>(0, 0, 0), true);   //第二个是它的location，接触点在父连杆中的位置
    } else {
      model.addBody(_kneeInertia, _kneeRotorInertia, _kneeGearRatio, bodyID - 1,
                    JointType::Revolute, CoordinateAxis::Y, xtreeKnee,
                    xtreeKneeRotor);
      model.addGroundContactPoint(bodyID, Vec3<T>(0 ,0, 0), true);     //第二个是它的location，接触点在该坐标系中的位置
    }
    // Wheel Joint
    bodyID++;
    Mat6<T> xtreeWheel = createSXform(I3, withLegSigns<T>(_wheelLocation,legID));                      
    Mat6<T> xtreeWheelRotor = createSXform(I3,withLegSigns<T>(_wheelRotorLocation,legID));

    if (sideSign > 1) {
      model.addBody(_wheelInertia.flipAlongAxis(CoordinateAxis::Y),         //翻转轴
                    _wheelRotorInertia.flipAlongAxis(CoordinateAxis::Y),
                    _wheelGearRatio, bodyID - 1, JointType::Revolute,
                    CoordinateAxis::Y, xtreeWheel, xtreeWheelRotor);
      model.addGroundContactPoint(bodyID, Vec3<T>(0, 0, 0), true);   
    } else {
      model.addBody(_wheelInertia, _wheelRotorInertia, _wheelGearRatio, bodyID - 1,
                    JointType::Revolute, CoordinateAxis::Y, xtreeWheel,
                    xtreeWheelRotor);

      model.addGroundContactPoint(bodyID, Vec3<T>(0, 0, 0), true);
    }
    sideSign++;
  }
  // 
  Vec3<T> g(0, 0, -9.81);
  model.setGravity(g);

  return true;
}

/*!
 * Build a QuadrupedRobot of the quadruped
 */
template <typename T>
QuadrupedRobot<T> Quadruped<T>::buildModel() {
  QuadrupedRobot<T> model;
  buildModel(model);
  return model;
}

/*!
 * Flip signs of elements of a vector V depending on which leg it belongs to
 根据腿部编号调整向量方向
 */
template <typename T, typename T2>
Vec3<T> withLegSigns(const Eigen::MatrixBase<T2>& v, int legID) {
  static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3,
                "Must have 3x1 matrix");
  switch (legID) {
    case 0:
      return Vec3<T>(v[0], v[1], v[2]);
    case 1:
      return Vec3<T>(v[0], v[1], v[2]);
    case 2:
      return Vec3<T>(v[0], -v[1], v[2]);
    case 3:
      return Vec3<T>(v[0], -v[1], v[2]);
    default:
      throw std::runtime_error("Invalid leg id!");
  }
}

template <typename T, typename T2>
Vec3<T> withLegSignsHip(const Eigen::MatrixBase<T2>& v, int legID) {
  static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3,
                "Must have 3x1 matrix");
  switch (legID) {
    case 0:
      return Vec3<T>(v[0], v[1], v[2]);
    case 1:
      return Vec3<T>(-v[0], v[1], v[2]);
    case 2:
      return Vec3<T>(-v[0], -v[1], v[2]);
    case 3:
      return Vec3<T>(v[0], -v[1], v[2]);
    default:
      throw std::runtime_error("Invalid leg id!");
  }
}


template class Quadruped<double>;
template class Quadruped<float>;
