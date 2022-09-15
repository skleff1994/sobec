///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2020, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include <crocoddyl/core/utils/exception.hpp>
#include <crocoddyl/core/utils/math.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/contact-dynamics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include "dam-augmented.hpp"

namespace sobec {

template <typename Scalar>
DAMSoftContactAbstractAugmentedFwdDynamicsTpl<Scalar>::DAMSoftContactAbstractAugmentedFwdDynamicsTpl(
    boost::shared_ptr<StateMultibody> state, 
    boost::shared_ptr<ActuationModelAbstract> actuation,
    boost::shared_ptr<CostModelSum> costs,
    const pinocchio::FrameIndex frameId,
    const Scalar Kp, 
    const Scalar Kv,
    const Vector3s& oPc,
    const std::size_t nc,
    const pinocchio::ReferenceFrame ref)
    : Base(state, actuation, costs) {
  if (this->get_costs()->get_nu() != this->get_nu()) {
    throw_pretty("Invalid argument: "
                 << "Costs doesn't have the same control dimension (it should be " + std::to_string(this->get_nu()) + ")");
  }
  Base::set_u_lb(Scalar(-1.) * this->get_pinocchio().effortLimit.tail(this->get_nu()));
  Base::set_u_ub(Scalar(+1.) * this->get_pinocchio().effortLimit.tail(this->get_nu()));
  // Soft contact model parameters
  if(Kp_ < 0.){
     throw_pretty("Invalid argument: "
                << "Kp_ must be positive "); 
  }
  if(Kv_ < 0.){
     throw_pretty("Invalid argument: "
                << "Kv_ must be positive "); 
  }
  Kp_ = Kp;
  Kv_ = Kv;
  oPc_ = oPc;
  frameId_ = frameId;
  ref_ = ref;
  with_force_cost_ = false;
  active_contact_ = true;
  nc_ = nc;
  parentId_ = this->get_pinocchio().frames[frameId_].parent;
  jMf_ = this->get_pinocchio().frames[frameId_].placement;
  with_armature_ = false;
  armature_ = VectorXs::Zero(this->get_state()->get_nv());
}

template <typename Scalar>
DAMSoftContactAbstractAugmentedFwdDynamicsTpl<Scalar>::~DAMSoftContactAbstractAugmentedFwdDynamicsTpl() {}


template <typename Scalar>
void DAMSoftContactAbstractAugmentedFwdDynamicsTpl<Scalar>::calc(
                const boost::shared_ptr<DifferentialActionDataAbstract>& data, 
                const Eigen::Ref<const VectorXs>& x,
                const Eigen::Ref<const VectorXs>& f,
                const Eigen::Ref<const VectorXs>& u) {
  if (static_cast<std::size_t>(x.size()) != this->get_state()->get_nx()) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " + std::to_string(this->get_state()->get_nx()) + ")");
  }
  if (static_cast<std::size_t>(f.size()) != this->get_nc()) {
    throw_pretty("Invalid argument: "
                 << "f has wrong dimension (it should be 3)");
  }
  if (static_cast<std::size_t>(u.size()) != this->get_nu()) {
    throw_pretty("Invalid argument: "
                 << "u has wrong dimension (it should be " + std::to_string(this->get_nu()) + ")");
  }
}

template <typename Scalar>
void DAMSoftContactAbstractAugmentedFwdDynamicsTpl<Scalar>::calc(
                const boost::shared_ptr<DifferentialActionDataAbstract>& data, 
                const Eigen::Ref<const VectorXs>& x,
                const Eigen::Ref<const VectorXs>& f) {
  if (static_cast<std::size_t>(x.size()) != this->get_state()->get_nx()) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " + std::to_string(this->get_state()->get_nx()) + ")");
  }
  if (static_cast<std::size_t>(f.size()) != this->get_nc()) {
    throw_pretty("Invalid argument: "
                 << "f has wrong dimension (it should be 3)");
  }
}

template <typename Scalar>
void DAMSoftContactAbstractAugmentedFwdDynamicsTpl<Scalar>::calcDiff(
                const boost::shared_ptr<DifferentialActionDataAbstract>& data, 
                const Eigen::Ref<const VectorXs>& x,
                const Eigen::Ref<const VectorXs>& f,
                const Eigen::Ref<const VectorXs>& u) {
  if (static_cast<std::size_t>(x.size()) != this->get_state()->get_nx()) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " + std::to_string(this->get_state()->get_nx()) + ")");
  }
  if (static_cast<std::size_t>(f.size()) != this->get_nc()) {
    throw_pretty("Invalid argument: "
                 << "f has wrong dimension (it should be 3)");
  }
  if (static_cast<std::size_t>(u.size()) != this->get_nu()) {
    throw_pretty("Invalid argument: "
                 << "u has wrong dimension (it should be " + std::to_string(this->get_nu()) + ")");
  }
}

template <typename Scalar>
void DAMSoftContactAbstractAugmentedFwdDynamicsTpl<Scalar>::calcDiff(
                const boost::shared_ptr<DifferentialActionDataAbstract>& data, 
                const Eigen::Ref<const VectorXs>& x,
                const Eigen::Ref<const VectorXs>& f) {
  if (static_cast<std::size_t>(x.size()) != this->get_state()->get_nx()) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " + std::to_string(this->get_state()->get_nx()) + ")");
  }
  if (static_cast<std::size_t>(f.size()) != this->get_nc()) {
    throw_pretty("Invalid argument: "
                 << "f has wrong dimension (it should be 3)");
  }
}




template <typename Scalar>
boost::shared_ptr<DifferentialActionDataAbstractTpl<Scalar> >
DAMSoftContactAbstractAugmentedFwdDynamicsTpl<Scalar>::createData() {
  return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this);
}


template <typename Scalar>
void DAMSoftContactAbstractAugmentedFwdDynamicsTpl<Scalar>::set_Kp(const Scalar inKp) {
  if (inKp < 0.) {
    throw_pretty("Invalid argument: "
                 << "Stiffness should be positive");
  }
  Kp_ = inKp;
}

template <typename Scalar>
void DAMSoftContactAbstractAugmentedFwdDynamicsTpl<Scalar>::set_Kv(const Scalar inKv) {
  if (inKv < 0.) {
    throw_pretty("Invalid argument: "
                 << "Damping should be positive");
  }
  Kv_ = inKv;
}

template <typename Scalar>
void DAMSoftContactAbstractAugmentedFwdDynamicsTpl<Scalar>::set_oPc(const Vector3s& inoPc) {
  if (inoPc.size() != 3) {
    throw_pretty("Invalid argument: "
                 << "Anchor point position should have size 3");
  }
  oPc_ = inoPc;
}


template <typename Scalar>
void DAMSoftContactAbstractAugmentedFwdDynamicsTpl<Scalar>::set_ref(const pinocchio::ReferenceFrame inRef) {
  ref_ = inRef;
}

template <typename Scalar>
void DAMSoftContactAbstractAugmentedFwdDynamicsTpl<Scalar>::set_id(const pinocchio::FrameIndex inId) {
  frameId_ = inId;
}

template <typename Scalar>
const Scalar DAMSoftContactAbstractAugmentedFwdDynamicsTpl<Scalar>::get_Kp() const {
  return Kp_;
}

template <typename Scalar>
const Scalar DAMSoftContactAbstractAugmentedFwdDynamicsTpl<Scalar>::get_Kv() const {
  return Kv_;
}

template <typename Scalar>
const typename MathBaseTpl<Scalar>::Vector3s& DAMSoftContactAbstractAugmentedFwdDynamicsTpl<Scalar>::get_oPc() const {
  return oPc_;
}

template <typename Scalar>
const pinocchio::ReferenceFrame& DAMSoftContactAbstractAugmentedFwdDynamicsTpl<Scalar>::get_ref() const {
  return ref_;
}

template <typename Scalar>
const pinocchio::FrameIndex& DAMSoftContactAbstractAugmentedFwdDynamicsTpl<Scalar>::get_id() const {
  return frameId_;
}


// armature
template <typename Scalar>
const typename MathBaseTpl<Scalar>::VectorXs& DAMSoftContactAbstractAugmentedFwdDynamicsTpl<Scalar>::get_armature() const {
  return armature_;
}

template <typename Scalar>
void DAMSoftContactAbstractAugmentedFwdDynamicsTpl<Scalar>::set_armature(const VectorXs& armature) {
  if (static_cast<std::size_t>(armature.size()) != this->get_state()->get_nv()) {
    throw_pretty("Invalid argument: "
                 << "The armature dimension is wrong (it should be " + std::to_string(this->get_state()->get_nv()) + ")");
  }
  armature_ = armature;
  with_armature_ = true;
}


}  // namespace sobec
