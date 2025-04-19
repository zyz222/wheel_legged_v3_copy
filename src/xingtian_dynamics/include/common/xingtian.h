/*! @file xingtian.h
 *  @brief Utility function to build a xingtian Quadruped object
 *
 *  This file is based on Cheetah3FullRotorModel_mex.m (originally written by
 * Pat) and builds a model of the Cheetah 3 robot.  The inertia parameters of
 * legs and rotors were determined through an experimental procedure described
 * in "Linear Matrix Inequalities for Physically-Consistent Inertial Parameter
 *      Identification: A Statistical Perspective on the Mass Distribution" by
 * Wensing, Kim, Slotine. (see https://arxiv.org/abs/1701.04395)
 *
 *  It turns out that the parameters are not fully observable when the base is
 * fixed, which is described in "Observability in Inertial Parameter
 * Identification" by Wensing, Niemeyer, Slotine (see
 * https://arxiv.org/abs/1711.03896).
 *
 *  However, these estimates are still very good and were confirmed against a
 * CAD model of the robot.
 */

#ifndef C22BB37D_9E63_4170_9332_804CA196B16F
#define C22BB37D_9E63_4170_9332_804CA196B16F

#ifndef LIBBIOMIMETICS_CHEETAH3_H
#define LIBBIOMIMETICS_CHEETAH3_H

#include "common/spatial.h"
#include "common/xingtianrobot.h"
#include "common/Quadruped.h"
#include "common/cppTypes.h"

using namespace spatial;

/*!
 * Generate a Quadruped model of Cheetah 3
 */
template <typename T>
Quadruped<T> buildXingtian() {
  Quadruped<T> xingtian;
  xingtian._robotType = RobotType::xingtian;

  xingtian._bodyMass = 26.135;
  // xingtian._bodyLength = 0.802;   //机身的长度
  xingtian._bodyLength = 0.418;
  xingtian._bodyWidth = 0.334;
  // xingtian._bodyHeight = 0.224;
  xingtian._bodyHeight = 0.072;
  xingtian._wheelGearRatio = 1;
  xingtian._hipGearRatio = 1;
  xingtian._kneeGearRatio = 1;
  xingtian._wheelLinkLength = 0;
 
  xingtian._hipLinkLength = 0.14;
  xingtian._kneeLinkLength = 0.14;

  xingtian._kneeLinkY_offset = 0.021;
  xingtian._wheelLinkY_offset = 0.013;       //轮子相对于父连杆的偏移
  
  xingtian._maxLegLength = 0.28;

  // xingtian._batteryV = 65;
  // xingtian._motorKT = 0.266;
  // xingtian._motorR = 0.45;
  // xingtian._jointDamping = .1;
  // xingtian._jointDryFriction = 1;
  // xingtian._motorTauMax = 27.2;

  MassProperties<T> wheelMassProperties, hipMassProperties, kneeMassProperties,
      wheelRotorMassProperties, hipRotorMassProperties, kneeRotorMassProperties;
    // 质量性能
  wheelMassProperties << 3.509,
        0.0,0.0, 0.0, 
        0.013094099,0.013976692, 0.013094099,
        0,0, 0;

  hipMassProperties << 0.25201, 
    -7.1495e-04,-2.17056e-02, -1.0000375583e-02, 
    0.00093214,0.001291752, 0.000410431,
     0,-0.000522990, 0;

  kneeMassProperties << 0.102, -1.11384e-04,
      2.090082e-03, -8.196822e-03, 0.000926259,
      0.000890565, 0.000051865, 0,
      0.000011884, 0;


    wheelRotorMassProperties.setZero();

    hipRotorMassProperties.setZero();

    kneeRotorMassProperties.setZero();
  // 转换成6*6空间惯性矩阵
  xingtian._wheelInertia = SpatialInertia<T>(wheelMassProperties);
  xingtian._hipInertia = SpatialInertia<T>(hipMassProperties);
  xingtian._kneeInertia = SpatialInertia<T>(kneeMassProperties);
  xingtian._wheelRotorInertia = SpatialInertia<T>(wheelRotorMassProperties);
  xingtian._hipRotorInertia = SpatialInertia<T>(hipRotorMassProperties);
  xingtian._kneeRotorInertia = SpatialInertia<T>(kneeRotorMassProperties);
  Vec3<T> bodyCOM(0, 0, 0);   //机身质心位置
  Vec3<T> bodyDims(xingtian._bodyLength, xingtian._bodyWidth,
                   xingtian._bodyHeight);       
  xingtian._bodyInertia = SpatialInertia<T>(                               //质量
      xingtian._bodyMass, bodyCOM, rotInertiaOfBox(xingtian._bodyMass, bodyDims));

  // this doesn't generalize to the mini xingtian?
  // xingtian._hipLocation =
  //                      Vec3<T>(xingtian._bodyLength, xingtian._bodyWidth, -0.036*2)*0.5 ;
  xingtian._hipLocation =Vec3<T>(0.418*0.5, 0.334*0.5, -0.036);    //左前腿在机身坐标系下的位置
  // note that this is wrong to match a small bug in the actual simulator, just
  // to test that I get the right answer. TODO fix!
  xingtian._hipRotorLocation = Vec3<T>(0.418*0.5, 0.334*0.5, -0.036);     //以当前的连杆坐标系为原点，来建立
  //都是以左前腿为正  ，这几个没问题！！
    


  xingtian._kneeLocation = Vec3<T>(xingtian._hipLinkLength, 0.021, 0);   //连杆位置坐标
  xingtian._kneeRotorLocation = Vec3<T>(xingtian._hipLinkLength, 0.021, 0);

  xingtian._wheelLocation = Vec3<T>(xingtian._kneeLinkLength, 0.013, 0);
  xingtian._wheelRotorLocation = Vec3<T>(xingtian._kneeLinkLength, 0.013, 0);

  // xingtian._kneeLocation = Vec3<T>(0, 0.021, 0);   //连杆位置坐标
  // xingtian._kneeRotorLocation = Vec3<T>(0, 0.021, 0);

  // xingtian._wheelLocation = Vec3<T>(0, 0.013, 0);
  // xingtian._wheelRotorLocation = Vec3<T>(0, 0.013, 0);

  // xingtian._hipRotorLocation = Vec3<T>(0, 0, 0);
  // xingtian._kneeRotorLocation = Vec3<T>(0, 0, 0);
  // xingtian._wheelRotorLocation = Vec3<T>(0, 0, 0);
  return xingtian;
}

#endif  // LIBBIOMIMETICS_CHEETAH3_H


#endif /* C22BB37D_9E63_4170_9332_804CA196B16F */
