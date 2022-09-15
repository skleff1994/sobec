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

#include "dam1d-augmented.hpp"

namespace sobec {

template <typename Scalar>
DAMSoftContact1DAugmentedFwdDynamicsTpl<Scalar>::DAMSoftContact1DAugmentedFwdDynamicsTpl(
    boost::shared_ptr<StateMultibody> state, 
    boost::shared_ptr<ActuationModelAbstract> actuation,
    boost::shared_ptr<CostModelSum> costs,
    const pinocchio::FrameIndex frameId,
    const double Kp, 
    const double Kv,
    const Vector3s& oPc,
    const pinocchio::ReferenceFrame ref,
    const Vector3MaskType& type)
    : Base(state, actuation, costs, frameId, Kp, Kv, oPc, 1, ref) {
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
  type_ = type;
  with_force_cost_ = false;
  active_contact_ = true;
  parentId_ = this->get_pinocchio().frames[frameId_].parent;
  jMf_ = this->get_pinocchio().frames[frameId_].placement;
  with_armature_ = false;
  armature_ = VectorXs::Zero(this->get_state()->get_nv());
}

template <typename Scalar>
DAMSoftContact1DAugmentedFwdDynamicsTpl<Scalar>::~DAMSoftContact1DAugmentedFwdDynamicsTpl() {}

template <typename Scalar>
void DAMSoftContact1DAugmentedFwdDynamicsTpl<Scalar>::calc(
            const boost::shared_ptr<DifferentialActionDataAbstract>& data, 
            const Eigen::Ref<const VectorXs>& x,
            const Eigen::Ref<const VectorXs>& f,
            const Eigen::Ref<const VectorXs>& u) {
  if (static_cast<std::size_t>(x.size()) != this->get_state()->get_nx()) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " + std::to_string(this->get_state()->get_nx()) + ")");
  }
  if (static_cast<std::size_t>(u.size()) != this->get_nu()) {
    throw_pretty("Invalid argument: "
                 << "u has wrong dimension (it should be " + std::to_string(this->get_nu()) + ")");
  }

  Data* d = static_cast<Data*>(data.get());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> q = x.head(this->get_state()->get_nq());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> v = x.tail(this->get_state()->get_nv());
  pinocchio::computeAllTerms(this->get_pinocchio(), d->pinocchio, q, v);
  pinocchio::updateFramePlacements(this->get_pinocchio(), d->pinocchio);
  d->oRf = d->pinocchio.oMf[frameId_].rotation();
  // Actuation calc
  this->get_actuation()->calc(d->multibody.actuation, x, u);
  
  // If contact is active, compute aq = ABA(q,v,tau,fext)
  if(active_contact_){
    // Compute external wrench for LOCAL f
    d->f3d = Vector3s::Zero();
    d->f3d(this->get_type()) = f(0);
    d->pinForce = pinocchio::ForceTpl<Scalar>(d->f3d, Vector3s::Zero());
    d->fext[parentId_] = jMf_.act(d->pinForce);
    // Copy for later
    d->fext_copy = d->fext;
    // Rotate if not f not in LOCAL
    if(ref_ != pinocchio::LOCAL){
        d->pinForce = pinocchio::ForceTpl<Scalar>(d->oRf.transpose() * d->f3d, Vector3s::Zero());
        d->fext[parentId_] = jMf_.act(d->pinForce);
    }

    // ABA with armature
    if(with_armature_){
      d->pinocchio.M.diagonal() += armature_;
      pinocchio::cholesky::decompose(this->get_pinocchio(), d->pinocchio);
      d->Minv.setZero();
      pinocchio::cholesky::computeMinv(this->get_pinocchio(), d->pinocchio, d->Minv);
      d->u_drift = d->multibody.actuation->tau - d->pinocchio.nle;
      //  Compute jacobian transpose lambda
      pinocchio::getFrameJacobian(this->get_pinocchio(), d->pinocchio, frameId_, pinocchio::LOCAL, d->lJ);
      d->xout.noalias() = d->Minv * d->u_drift + d->Minv * d->lJ.topRows(3).transpose() * d->pinForce.linear(); //d->JtF;
    // ABA without armature
    } else {
      d->xout = pinocchio::aba(this->get_pinocchio(), d->pinocchio, q, v, d->multibody.actuation->tau, d->fext); 
    }
    // Compute time derivative of contact force : need to forward kin with current acc
    pinocchio::forwardKinematics(this->get_pinocchio(), d->pinocchio, q, v, d->xout);
    d->la = pinocchio::getFrameAcceleration(this->get_pinocchio(), d->pinocchio, frameId_, pinocchio::LOCAL).linear();     
    d->lv = pinocchio::getFrameVelocity(this->get_pinocchio(), d->pinocchio, frameId_, pinocchio::LOCAL).linear();
    d->fout3d = -Kp_ * d->lv - Kv_ * d->la;
    d->fout(0) = d->fout3d(this->get_type());
    d->fout3d_copy = d->fout3d;
    d->fout_copy = d->fout;
    // Rotate if not f not in LOCAL
    if(ref_ != pinocchio::LOCAL){
        d->oa = pinocchio::getFrameAcceleration(this->get_pinocchio(), d->pinocchio, frameId_, pinocchio::LOCAL_WORLD_ALIGNED).linear();
        d->ov = pinocchio::getFrameVelocity(this->get_pinocchio(), d->pinocchio, frameId_, pinocchio::LOCAL_WORLD_ALIGNED).linear();
        d->fout3d = -Kp_* d->ov - Kv_ * d->oa;
        d->fout(0) = d->fout3d(this->get_type());
    } 
  }

  // If contact NOT active : compute aq = ABA(q,v,tau)
  else {
    if (with_armature_) {
      // pinocchio::computeAllTerms(this->get_pinocchio(), d->pinocchio, q, v);
      d->pinocchio.M.diagonal() += armature_;
      pinocchio::cholesky::decompose(this->get_pinocchio(), d->pinocchio);
      d->Minv.setZero();
      pinocchio::cholesky::computeMinv(this->get_pinocchio(), d->pinocchio, d->Minv);
      d->u_drift = d->multibody.actuation->tau - d->pinocchio.nle;
      d->xout.noalias() = d->Minv * d->u_drift;
    } else {
      d->xout = pinocchio::aba(this->get_pinocchio(), d->pinocchio, q, v, d->multibody.actuation->tau);
    }
  }

  pinocchio::updateGlobalPlacements(this->get_pinocchio(), d->pinocchio);
  
  // Computing the cost value and residuals
  this->get_costs()->calc(d->costs, x, u);
  d->cost = d->costs->cost;

  // Add hard-coded cost on contact force
  if(with_force_cost_){
    d->f_residual = f - force_des_;
    d->cost += 0.5* force_weight_ * d->f_residual.transpose() * d->f_residual;
  }
}


template <typename Scalar>
void DAMSoftContact1DAugmentedFwdDynamicsTpl<Scalar>::calc(
            const boost::shared_ptr<DifferentialActionDataAbstract>& data, 
            const Eigen::Ref<const VectorXs>& x,
            const Eigen::Ref<const VectorXs>& f) {
  if (static_cast<std::size_t>(x.size()) != this->get_state()->get_nx()) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " + std::to_string(this->get_state()->get_nx()) + ")");
  }
  if (static_cast<std::size_t>(f.size()) != this->get_nc()) {
    throw_pretty("Invalid argument: "
                 << "f has wrong dimension (it should be " + std::to_string(this->get_nc()) + ")");
  }
  Data* d = static_cast<Data*>(data.get());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> q = x.head(this->get_state()->get_nq());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> v = x.tail(this->get_state()->get_nv());
  pinocchio::computeAllTerms(this->get_pinocchio(), d->pinocchio, q, v);
  this->get_costs()->calc(d->costs, x);
  d->cost = d->costs->cost;
  // Add cost on force here?
}



template <typename Scalar>
void DAMSoftContact1DAugmentedFwdDynamicsTpl<Scalar>::calcDiff(
    const boost::shared_ptr<DifferentialActionDataAbstract>& data, 
    const Eigen::Ref<const VectorXs>& x,
    const Eigen::Ref<const VectorXs>& f,
    const Eigen::Ref<const VectorXs>& u) {
  if (static_cast<std::size_t>(x.size()) != this->get_state()->get_nx()) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " + std::to_string(this->get_state()->get_nx()) + ")");
  }
  if (static_cast<std::size_t>(u.size()) != this->get_nu()) {
    throw_pretty("Invalid argument: "
                 << "u has wrong dimension (it should be " + std::to_string(this->get_nu()) + ")");
  }

  const std::size_t nv = this->get_state()->get_nv();
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> q = x.head(this->get_state()->get_nq());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> v = x.tail(nv);
  Data* d = static_cast<Data*>(data.get());
  d->oRf = d->pinocchio.oMf[frameId_].rotation();
  // Actuation calcDiff
  this->get_actuation()->calcDiff(d->multibody.actuation, x, u);
  
  // If contact is active, compute ABA derivatives + force
  if(active_contact_){
    // Compute Jacobian
    // pinocchio::framesForwardKinematics(this->get_pinocchio(), d->pinocchio, q);
    pinocchio::getFrameJacobian(this->get_pinocchio(), d->pinocchio, frameId_, pinocchio::LOCAL, d->lJ);

    // Derivatives of d->xout (ABA) w.r.t. x and u in LOCAL (same in WORLD)
    // No armature
    if(!with_armature_){
      pinocchio::computeABADerivatives(this->get_pinocchio(), d->pinocchio, q, v, d->multibody.actuation->tau, d->fext, 
                                                                  d->aba_dq, d->aba_dv, d->aba_dtau);
      d->Fx.leftCols(nv) = d->aba_dq;
      d->Fx.rightCols(nv) = d->aba_dv; 
      d->Fx += d->aba_dtau * d->multibody.actuation->dtau_dx;
      d->Fu = d->aba_dtau * d->multibody.actuation->dtau_du;
      // Compute derivatives of d->xout (ABA) w.r.t. f in LOCAL 
      d->aba_df3d = d->aba_dtau * d->lJ.topRows(3).transpose() * jMf_.rotation() * Matrix3s::Identity();
      d->aba_df3d_copy = d->aba_df3d;
      d->aba_df = d->aba_df3d.col(this->get_type());
      // Skew term added to RNEA derivatives when force is expressed in LWA
      if(ref_ != pinocchio::LOCAL){
          d->Fx.leftCols(nv)+= d->aba_dtau * d->lJ.topRows(3).transpose() * pinocchio::skew(d->oRf.transpose() * d->f3d_copy) * d->lJ.bottomRows(3);
          // Rotate dABA/df
          d->aba_df3d = d->aba_df3d_copy * d->oRf.transpose();
          d->aba_df = d->aba_df3d.col(this->get_type());
      }
    // With armature
    } else {
        pinocchio::computeRNEADerivatives(this->get_pinocchio(), d->pinocchio, q, v, d->xout, d->fext);
        d->dtau_dx.leftCols(nv) = d->multibody.actuation->dtau_dx.leftCols(nv) - d->pinocchio.dtau_dq;
        d->dtau_dx.rightCols(nv) = d->multibody.actuation->dtau_dx.rightCols(nv) - d->pinocchio.dtau_dv;
        d->Fx.noalias() = d->Minv * d->dtau_dx;
        d->Fu.noalias() = d->Minv * d->multibody.actuation->dtau_du;
        // Compute derivatives of d->xout (ABA) w.r.t. f in LOCAL 
        d->aba_df3d = d->Minv * d->lJ.topRows(3).transpose() * jMf_.rotation() * Matrix3s::Identity();
        d->aba_df3d_copy = d->aba_df3d;
        d->aba_df = d->aba_df3d.col(this->get_type());
        // Skew term added to RNEA derivatives when force is expressed in LWA
        if(ref_ != pinocchio::LOCAL){
            d->Fx.leftCols(nv)+= d->Minv * d->lJ.topRows(3).transpose() * pinocchio::skew(d->oRf.transpose() * d->f3d_copy) * d->lJ.bottomRows(3);
            // Rotate dABA/df
          d->aba_df3d = d->aba_df3d_copy * d->oRf.transpose();
          d->aba_df = d->aba_df3d.col(this->get_type());
        }
      }

    // Derivatives of d->fout in LOCAL : important >> UPDATE FORWARD KINEMATICS with d->xout
    pinocchio::getFrameVelocityDerivatives(this->get_pinocchio(), d->pinocchio, frameId_, pinocchio::LOCAL, 
                                                    d->lv_dq, d->lv_dv);
    d->lv_dx.leftCols(nv) = d->lv_dq;
    d->lv_dx.rightCols(nv) = d->lv_dv;
    // Derivatives of spatial acc w.r.t. (x, f, u)
    pinocchio::getFrameAccelerationDerivatives(this->get_pinocchio(), d->pinocchio, frameId_, pinocchio::LOCAL, 
                                                    d->v_dv, d->a_dq, d->a_dv, d->a_da);
    d->da_dx.topRows(3).leftCols(nv) = d->a_dq.topRows(3) + d->a_da.topRows(3) * d->Fx.leftCols(nv); 
    d->da_dx.topRows(3).rightCols(nv) = d->a_dv.topRows(3) + d->a_da.topRows(3) * d->Fx.rightCols(nv); 
    d->da_du.topRows(3) = d->a_da.topRows(3) * d->Fu;
    d->da_df3d.topRows(3) = d->a_da.topRows(3) * d->aba_df3d;
    d->da_df.topRows(3) = d->da_df3d.topRows(3).col(this->get_type());
    // Derivatives of fdot w.r.t. (x,f,u)
    d->dfdt3d_dx = -Kp_*d->lv_dx.topRows(3) - Kv_*d->da_dx.topRows(3);
    d->dfdt3d_du = -Kv_*d->da_du.topRows(3);
    d->dfdt3d_df = -Kv_*d->aba_df3d.topRows(3);
    d->dfdt_dx = d->dfdt3d_dx.row(this->get_type());
    d->dfdt_du = d->dfdt3d_du.row(this->get_type());
    d->dfdt_df = d->dfdt3d_df.row(this->get_type());
    d->dfdt3d_dx_copy = d->dfdt3d_dx;
    d->dfdt3d_du_copy = d->dfdt3d_du;
    d->dfdt3d_df_copy = d->dfdt3d_df;
    //Rotate dfout_dx if not LOCAL 
    if(ref_ != pinocchio::LOCAL){
        pinocchio::getFrameJacobian(this->get_pinocchio(), d->pinocchio, frameId_, pinocchio::LOCAL_WORLD_ALIGNED, d->oJ);
        d->dfdt3d_dx.leftCols(nv) = d->oRf * d->dfdt3d_dx_copy.leftCols(nv)- pinocchio::skew(d->oRf * d->fout3d_copy) * d->oJ.bottomRows(3);
        d->dfdt3d_dx.rightCols(nv) = d->oRf * d->dfdt3d_dx_copy.rightCols(nv);
        d->dfdt3d_du = d->oRf * d->dfdt3d_du_copy;
        d->dfdt3d_df = d->oRf * d->dfdt3d_df_copy;
        d->dfdt_dx = d->dfdt3d_dx.row(this->get_type());
        d->dfdt_du = d->dfdt3d_du.row(this->get_type());
        d->dfdt_df = d->dfdt3d_df.row(this->get_type());
    }
  }
  else {
    // Computing the dynamics derivatives
    if (!with_armature_) {
      // Computing the free forward dynamics with ABA derivatives
      pinocchio::computeABADerivatives(this->get_pinocchio(), d->pinocchio, q, v, d->multibody.actuation->tau, 
                                                      d->aba_dq, d->aba_dv, d->aba_dtau);
      d->Fx.leftCols(nv) = d->aba_dq;
      d->Fx.rightCols(nv) = d->aba_dv;
      d->Fx += d->aba_dtau * d->multibody.actuation->dtau_dx;
      d->Fu = d->aba_dtau * d->multibody.actuation->dtau_du;
    } else {
      pinocchio::computeRNEADerivatives(this->get_pinocchio(), d->pinocchio, q, v, d->xout);
      d->dtau_dx.leftCols(nv) = d->multibody.actuation->dtau_dx.leftCols(nv) - d->pinocchio.dtau_dq;
      d->dtau_dx.rightCols(nv) = d->multibody.actuation->dtau_dx.rightCols(nv) - d->pinocchio.dtau_dv;
      d->Fx.noalias() = d->Minv * d->dtau_dx;
      d->Fu.noalias() = d->Minv * d->multibody.actuation->dtau_du;
    }
  }

  this->get_costs()->calcDiff(d->costs, x, u);
  d->Lx = d->costs->Lx;
  d->Lu = d->costs->Lu;
  d->Lxx = d->costs->Lxx;
  d->Lxu = d->costs->Lxu;
  d->Luu = d->costs->Luu;
  // add hard-coded cost
  if(active_contact_ && with_force_cost_){
      d->f_residual = f - force_des_;
      d->Lf = force_weight_ * d->f_residual;
      d->Lff(0,0) = force_weight_;
  }
}


template <typename Scalar>
void DAMSoftContact1DAugmentedFwdDynamicsTpl<Scalar>::calcDiff(
    const boost::shared_ptr<DifferentialActionDataAbstract>& data, 
    const Eigen::Ref<const VectorXs>& x,
    const Eigen::Ref<const VectorXs>& f) {
  if (static_cast<std::size_t>(x.size()) != this->get_state()->get_nx()) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " + std::to_string(this->get_state()->get_nx()) + ")");
  }
  if (static_cast<std::size_t>(f.size()) != this->get_nc()) {
    throw_pretty("Invalid argument: "
                 << "f has wrong dimension (it should be " + std::to_string(this->get_nc()) + ")");
  }
  Data* d = static_cast<Data*>(data.get());
  this->get_costs()->calcDiff(d->costs, x);
  // Add cost on force here
}


template <typename Scalar>
boost::shared_ptr<DifferentialActionDataAbstractTpl<Scalar> >
DAMSoftContact1DAugmentedFwdDynamicsTpl<Scalar>::createData() {
  return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this);
}

template <typename Scalar>
void DAMSoftContact1DAugmentedFwdDynamicsTpl<Scalar>::set_force_cost(const VectorXs& inForceDes, 
                                                                     const Scalar inForceWeight) {
  if (inForceWeight < 0.) {
    throw_pretty("Invalid argument: "
                 << "Force weight should be positive");
  }
  if (static_cast<std::size_t>(inForceDes.size()) != this->get_nc()) {
    throw_pretty("Invalid argument: "
                 << "f has wrong dimension (it should be " + std::to_string(this->get_nc()) + ")");
  }
  force_des_ = inForceDes;
  force_weight_ = inForceWeight;
  with_force_cost_ = true;
}

// template <typename Scalar>
// void DAMSoftContact1DAugmentedFwdDynamicsTpl<Scalar>::set_Kp(const Scalar inKp) {
//   if (inKp < 0.) {
//     throw_pretty("Invalid argument: "
//                  << "Stiffness should be positive");
//   }
//   Kp_ = inKp;
// }

// template <typename Scalar>
// void DAMSoftContact1DAugmentedFwdDynamicsTpl<Scalar>::set_Kv(const Scalar inKv) {
//   if (inKv < 0.) {
//     throw_pretty("Invalid argument: "
//                  << "Damping should be positive");
//   }
//   Kv_ = inKv;
// }

// template <typename Scalar>
// void DAMSoftContact1DAugmentedFwdDynamicsTpl<Scalar>::set_oPc(const Vector3s& inoPc) {
//   if (inoPc.size() != 3) {
//     throw_pretty("Invalid argument: "
//                  << "Anchor point position should have size 3");
//   }
//   oPc_ = inoPc;
// }

template <typename Scalar>
void DAMSoftContact1DAugmentedFwdDynamicsTpl<Scalar>::set_force_des(const VectorXs& inForceDes) {
  if (static_cast<std::size_t>(inForceDes.size()) != this->get_nc()) {
    throw_pretty("Invalid argument: "
                 << "f has wrong dimension (it should be " + std::to_string(this->get_nc()) + ")");
  }
  force_des_ = inForceDes;
}

template <typename Scalar>
void DAMSoftContact1DAugmentedFwdDynamicsTpl<Scalar>::set_force_weight(const Scalar inForceWeight) {
  if (inForceWeight < 0.) {
    throw_pretty("Invalid argument: "
                 << "Force cost weight should be positive");
  }
  force_weight_ = inForceWeight;
}

// template <typename Scalar>
// void DAMSoftContact1DAugmentedFwdDynamicsTpl<Scalar>::set_ref(const pinocchio::ReferenceFrame inRef) {
//   ref_ = inRef;
// }

// template <typename Scalar>
// void DAMSoftContact1DAugmentedFwdDynamicsTpl<Scalar>::set_id(const pinocchio::FrameIndex inId) {
//   frameId_ = inId;
// }

// template <typename Scalar>
// const Scalar DAMSoftContact1DAugmentedFwdDynamicsTpl<Scalar>::get_Kp() const {
//   return Kp_;
// }

// template <typename Scalar>
// const Scalar DAMSoftContact1DAugmentedFwdDynamicsTpl<Scalar>::get_Kv() const {
//   return Kv_;
// }

// template <typename Scalar>
// const typename MathBaseTpl<Scalar>::Vector3s& DAMSoftContact1DAugmentedFwdDynamicsTpl<Scalar>::get_oPc() const {
//   return oPc_;
// }

template <typename Scalar>
const typename MathBaseTpl<Scalar>::VectorXs& DAMSoftContact1DAugmentedFwdDynamicsTpl<Scalar>::get_force_des() const {
  return force_des_;
}

template <typename Scalar>
const Scalar DAMSoftContact1DAugmentedFwdDynamicsTpl<Scalar>::get_force_weight() const {
  return force_weight_;
}

template <typename Scalar>
const Vector3MaskType& DAMSoftContact1DAugmentedFwdDynamicsTpl<Scalar>::get_type() const {
  return type_;
}

template <typename Scalar>
void DAMSoftContact1DAugmentedFwdDynamicsTpl<Scalar>::set_type(const Vector3MaskType& inType) {
  type_ = inType;
}

// template <typename Scalar>
// const pinocchio::ReferenceFrame& DAMSoftContact1DAugmentedFwdDynamicsTpl<Scalar>::get_ref() const {
//   return ref_;
// }

// template <typename Scalar>
// const pinocchio::FrameIndex& DAMSoftContact1DAugmentedFwdDynamicsTpl<Scalar>::get_id() const {
//   return frameId_;
// }


// // armature
// template <typename Scalar>
// const typename MathBaseTpl<Scalar>::VectorXs& DAMSoftContact1DAugmentedFwdDynamicsTpl<Scalar>::get_armature() const {
//   return armature_;
// }

// template <typename Scalar>
// void DAMSoftContact1DAugmentedFwdDynamicsTpl<Scalar>::set_armature(const VectorXs& armature) {
//   if (static_cast<std::size_t>(armature.size()) != this->get_state()->get_nv()) {
//     throw_pretty("Invalid argument: "
//                  << "The armature dimension is wrong (it should be " + std::to_string(this->get_state()->get_nv()) + ")");
//   }
//   armature_ = armature;
//   with_armature_ = true;
// }


}  // namespace sobec
