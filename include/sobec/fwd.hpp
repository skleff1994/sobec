///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2021, LAAS-CNRS, University of Edinburgh, INRIA
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef SOBEC_FWD_HPP_
#define SOBEC_FWD_HPP_

namespace sobec {

// Cost COM-vel
template <typename Scalar>
class ResidualModelCoMVelocityTpl;
template <typename Scalar>
struct ResidualDataCoMVelocityTpl;
typedef ResidualModelCoMVelocityTpl<double> ResidualModelCoMVelocity;
typedef ResidualDataCoMVelocityTpl<double> ResidualDataCoMVelocity;

// Cost COP
template <typename Scalar>
class ResidualModelCenterOfPressureTpl;
template <typename Scalar>
struct ResidualDataCenterOfPressureTpl;
typedef ResidualModelCenterOfPressureTpl<double> ResidualModelCenterOfPressure;
typedef ResidualDataCenterOfPressureTpl<double> ResidualDataCenterOfPressure;

// Cost velocity collision
template <typename Scalar>
class ResidualModelVelCollisionTpl;
template <typename Scalar>
struct ResidualDataVelCollisionTpl;
typedef ResidualModelVelCollisionTpl<double> ResidualModelVelCollision;
typedef ResidualDataVelCollisionTpl<double> ResidualDataVelCollision;

// Cost fly high
template <typename Scalar>
class ResidualModelFlyHighTpl;
template <typename Scalar>
struct ResidualDataFlyHighTpl;
typedef ResidualModelFlyHighTpl<double> ResidualModelFlyHigh;
typedef ResidualDataFlyHighTpl<double> ResidualDataFlyHigh;

// Activation quad-ref
template <typename Scalar>
class ActivationModelQuadRefTpl;
typedef ActivationModelQuadRefTpl<double> ActivationModelQuadRef;

}  // namespace sobec

#endif  // SOBEC_FWD_HPP_
