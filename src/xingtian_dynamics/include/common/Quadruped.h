/*! @file Quadruped.h
 *  @brief Data structure containing parameters for quadruped robot
 *
 *  This file contains the Quadruped class.  This stores all the parameters for
 * a quadruped robot.  There are utility functions to generate Quadruped objects
 * for Cheetah 3 (and eventually mini-cheetah). There is a buildModel() method
 * which can be used to create a floating-base dynamics model of the quadruped.
 */

#ifndef AC20F2A7_6E0B_4560_A647_29B90250205F
#define AC20F2A7_6E0B_4560_A647_29B90250205F

#ifndef LIBBIOMIMETICS_QUADRUPED_H
#define LIBBIOMIMETICS_QUADRUPED_H

#include <vector>
#include "common/xingtianrobot.h"
#include "common/SpatialInertia.h"
#include <eigen3/Eigen/StdVector>

/*!
 * Basic parameters for a cheetah-shaped robot
 */
namespace xingtian{
constexpr size_t num_act_joint = 12;      //执行器有12个
constexpr size_t num_q = 19;                
constexpr size_t dim_config = 18;             //所有的自由度
constexpr size_t num_leg = 4;
constexpr size_t num_leg_joint = 3;  //关节数3个
}  // namespace xingtian

/*!
 * Link indices for cheetah-shaped robots
 */
namespace linkID {
 //这个修改之后没问题,禁止修改
constexpr size_t FL = 8;  // Front Left Foot  //足端接触点   10
constexpr size_t HL = 11;  // Hind Left Foot                13
constexpr size_t HR = 14;  // Hind Right Foot               16
constexpr size_t FR = 17;   // Front Right Foot              19
//下边这个就不对了
constexpr size_t FL_hip = 6;  // Front Left Abduction   机身的左上顶点 7    link 左前hip 6
constexpr size_t HL_hip = 9;  // Hind Left Abduction    左下   10
constexpr size_t HR_hip = 12;  // Hind Right Abduction   右下   13
constexpr size_t FR_hip = 15;  // Front Right Abduction  右上   16
}  // namespace linkID

using std::vector;

/*!
 * Representation of a quadruped robot's physical properties.
 *
 * When viewed from the top, the quadruped's legs are:
 *  cheetah
 * FRONT
 * 2 1   RIGHT
 * 4 3
 * BACK
 * 
 * xingtian
 * FRONT
 * 1 4    RIGHT
 * 2 3
 * BACK
 *
 */
template <typename T>
class Quadruped {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RobotType _robotType;
  T _bodyLength, _bodyWidth, _bodyHeight, _bodyMass;
  T  _hipGearRatio, _kneeGearRatio,_wheelGearRatio;
  T _wheelLinkLength, _hipLinkLength, _kneeLinkLength, _kneeLinkY_offset,_wheelLinkY_offset, _maxLegLength;
  T _motorKT, _motorR, _batteryV;
  T _motorTauMax;
  T _jointDamping, _jointDryFriction;
  SpatialInertia<T> _wheelInertia, _hipInertia, _kneeInertia, _wheelRotorInertia,  //转子的转动惯量 都是0
      _hipRotorInertia, _kneeRotorInertia, _bodyInertia;
  Vec3<T> _wheelLocation, _wheelRotorLocation, _hipLocation, _hipRotorLocation,   //连杆和转子的位置
      _kneeLocation, _kneeRotorLocation;
  QuadrupedRobot<T> buildModel();
  bool buildModel(QuadrupedRobot<T>& model);


  /*!
   * Get if the i-th leg is on the left (+) or right (-) of the robot.
   * @param leg : the leg index
   * @return The side sign (-1 for right legs, +1 for left legs)
   */
  static T getSideSign(int leg) {
    const T sideSigns[4] = {1, 1, -1, -1};
    assert(leg >= 0 && leg < 4);
    return sideSigns[leg];
  }

  /*!
   * Get location of the hip for the given leg in robot frame
   * @param leg : the leg index
   */
  Vec3<T> getHipLocation(int leg) {
    assert(leg >= 0 && leg < 4);
    Vec3<T> pHip((leg == 0 || leg == 3) ? _hipLocation(0) : -_hipLocation(0),     //x
                 (leg == 0 || leg == 1) ? _hipLocation(1) : -_hipLocation(1),     //y
                 _hipLocation(2));                                                  //z
    return pHip;
  }
};

template <typename T, typename T2>
Vec3<T> withLegSigns(const Eigen::MatrixBase<T2>& v, int legID);

template <typename T, typename T2>
Vec3<T> withLegSignsHip(const Eigen::MatrixBase<T2>& v, int legID);
#endif  // LIBBIOMIMETICS_QUADRUPED_H


#endif /* AC20F2A7_6E0B_4560_A647_29B90250205F */
