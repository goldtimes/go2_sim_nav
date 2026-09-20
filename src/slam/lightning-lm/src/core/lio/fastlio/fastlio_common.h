#pragma once
#ifndef FASTLIO_COMMON_H
#define FASTLIO_COMMON_H

#include <Eigen/Eigen>

#include "common/eigen_types.h"

namespace fastlio {

using V3D = lightning::Vec3d;
using M3D = lightning::Mat3d;
using V3F = lightning::Vec3f;
using M3F = lightning::Mat3f;

static constexpr double G_m_s2 = 9.81;

#define MD(a, b) Eigen::Matrix<double, (a), (b)>
#define VD(a) Eigen::Matrix<double, (a), 1>
#define MF(a, b) Eigen::Matrix<float, (a), (b)>
#define VF(a) Eigen::Matrix<float, (a), 1>

}  // namespace fastlio

#endif  // FASTLIO_COMMON_H
