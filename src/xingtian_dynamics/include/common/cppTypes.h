/*! @file cTypes.h
 *  @brief Common types that are only valid in C++
 *
 *  This file contains types which are only used in C++ code.  This includes
 * Eigen types, template types, aliases, ...
 */

#ifndef PROJECT_CPPTYPES_H
#define PROJECT_CPPTYPES_H

#include <vector>
#include "cTypes.h"
#include <eigen3/Eigen/Dense>
#include <cstddef>
// Rotation Matrix
template <typename T>
using RotMat = typename Eigen::Matrix<T, 3, 3>;

// 2x1 Vector
template <typename T>
using Vec2 = typename Eigen::Matrix<T, 2, 1>;

// 3x1 Vector
template <typename T>
using Vec3 = typename Eigen::Matrix<T, 3, 1>;

// 4x1 Vector
template <typename T>
using Vec4 = typename Eigen::Matrix<T, 4, 1>;
template <typename T>
using Vec5 = typename Eigen::Matrix<T, 5, 1>;

template <typename T>
using Vec8 = typename Eigen::Matrix<T, 8, 1>;

// 4x1 Integer Vector
using VecInt4 = typename Eigen::Matrix<int, 4, 1>;
// 6x1 Vector
template <typename T>
using Vec6 = Eigen::Matrix<T, 6, 1>;

// 10x1 Vector
template <typename T>
using Vec10 = Eigen::Matrix<T, 10, 1>;

// 12x1 Vector
template <typename T>
using Vec12 = Eigen::Matrix<T, 12, 1>;

// 18x1 Vector
template <typename T>
using Vec18 = Eigen::Matrix<T, 18, 1>;

// 28x1 vector
template <typename T>
using Vec28 = Eigen::Matrix<T, 28, 1>;

// Dynamic Length Vector
template <typename T>
using VecX = typename Eigen::Matrix<T, Eigen::Dynamic, 1>;


// Homogenous Matrix
template <typename T>
using HomoMat = typename Eigen::Matrix<T, 4, 4>;


// 3x3 Matrix
template <typename T>
using Mat3 = typename Eigen::Matrix<T, 3, 3>;

// 3x3 Identity Matrix
#define _I3 Eigen::MatrixXd::Identity(3, 3)
//2*2 matrix

// template <typename T>
// using I3 = Eigen::Matrix<T, 3, 3>;

// // 单位矩阵生成函数
// template <typename T>
// I3<T> Identity3x3() {
//     return I3<T>::Identity();
// }

template <typename T>
using Mat2 = typename Eigen::Matrix<T, 2, 2>;
// 4x1 Vector
template <typename T>
using Quat = typename Eigen::Matrix<T, 4, 1>;

// Spatial Vector (6x1, all subspaces)
template <typename T>
using SVec = typename Eigen::Matrix<T, 6, 1>;

// Spatial Transform (6x6)
template <typename T>
using SXform = typename Eigen::Matrix<T, 6, 6>;

// 6x6 Matrix
template <typename T>
using Mat6 = typename Eigen::Matrix<T, 6, 6>;

// 12x12 Matrix
template <typename T>
using Mat12 = typename Eigen::Matrix<T, 12, 12>;

// 18x18 Matrix
template <typename T>
using Mat18 = Eigen::Matrix<T, 18, 18>;

// 28x28 Matrix
template <typename T>
using Mat28 = Eigen::Matrix<T, 28, 28>;

// 3x4 Matrix
template <typename T>
using Mat34 = Eigen::Matrix<T, 3, 4>;

template <typename T>
using Vec34 = Eigen::Matrix<T, 3, 4>;

template <typename T>
using Vec24 = Eigen::Matrix<T, 2, 4>;

// 3x4 Matrix
template <typename T>
using Mat23 = Eigen::Matrix<T, 2, 3>;

template <typename T>
using Vec54 = Eigen::Matrix<T, 5, 4>;
// 4x4 Matrix
template <typename T>
using Mat4 = typename Eigen::Matrix<T, 4, 4>;

template <typename T>
using Mat5 = typename Eigen::Matrix<T, 5, 5>;
// 10x1 Vector
template <typename T>
using MassProperties = typename Eigen::Matrix<T, 10, 1>;

// 12x12 Identity Matrix
#define I12 Eigen::MatrixXd::Identity(12, 12)

// 18x18 Identity Matrix
#define I18 Eigen::MatrixXd::Identity(18, 18)

// Dynamically sized vector
template <typename T>
using DVec = typename Eigen::Matrix<T, Eigen::Dynamic, 1>;

// Dynamically sized matrix
template <typename T>
using DMat = typename Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

template <typename T>
using MatX = typename Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
// Dynamically sized matrix with spatial vector columns
template <typename T>
using D6Mat = typename Eigen::Matrix<T, 6, Eigen::Dynamic>;

// Dynamically sized matrix with cartesian vector columns
template <typename T>
using D3Mat = typename Eigen::Matrix<T, 3, Eigen::Dynamic>;

// std::vector (a list) of Eigen things
template <typename T>
using vectorAligned = typename std::vector<T, Eigen::aligned_allocator<T>>;

// enum class RobotType { xingtian };

template <typename T>
inline Vec34<T> vec12ToVec34(Vec12<T> vec12){
    Vec34<T> vec34;
    for(int i(0); i < 4; ++i){
        vec34.col(i) = vec12.segment(3*i, 3);
    }
    return vec34;
}
template <typename T>

inline Vec12<T> vec34ToVec12(Vec34<T> vec34){
    Vec12<T> vec12;
    for(int i(0); i < 4; ++i){
        vec12.segment(3*i, 3) = vec34.col(i);
    }
    return vec12;
}

/*!
 * Basic parameters for a xingtian-shaped robot
//  */
// namespace xingtian {
//     constexpr size_t num_act_joint = 12;      //执行器有12个
//     constexpr size_t num_q = 19;               //广义坐标系的数量
//     constexpr size_t dim_config = 18;             //所有的自由度
//     constexpr size_t num_leg = 4;
//     constexpr size_t num_leg_joint = 3;  //关节数3个
// }  // namespace xingtian
    
//     /*!
//      * Link indices for xingtian-shaped robots
//     */
// namespace linkID {
//     constexpr size_t FR = 9;   // Front Right Foot   //前右脚
//     constexpr size_t FL = 11;  // Front Left Foot    //前左脚
//     constexpr size_t HR = 13;  // Hind Right Foot    //后右脚
//     constexpr size_t HL = 15;  // Hind Left Foot     //后左脚

//     constexpr size_t FR_abd = 2;  // Front Right Abduction
//     constexpr size_t FL_abd = 0;  // Front Left Abduction
//     constexpr size_t HR_abd = 3;  // Hind Right Abduction
//     constexpr size_t HL_abd = 1;  // Hind Left Abduction
// }  // namespace linkID

#endif  // PROJECT_CPPTYPES_H
