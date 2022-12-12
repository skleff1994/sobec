///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2019, LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include <pinocchio/algorithm/joint-configuration.hpp>

#include "crocoddyl/core/utils/exception.hpp"
#include "state.hpp"

namespace sobec {
using namespace crocoddyl;

template <typename Scalar>
StateSoftContactTpl<Scalar>::StateSoftContactTpl(
    boost::shared_ptr<pinocchio::ModelTpl<Scalar> > model, std::size_t nc)
    : Base(model->nq + model->nv + nc, 2 * model->nv + nc),
      pinocchio_(model),
      nc_(nc),
      y0_(VectorXs::Zero(model->nq + model->nv + nc)) {
  // In a multibody system, we could define the first joint using Lie groups.
  // The current cases are free-flyer (SE3) and spherical (S03).
  // Instead simple represents any joint that can model within the Euclidean
  // manifold. The rest of joints use Euclidean algebra. We use this fact for
  // computing Jdiff.

  nv_ = model->nv;        // tangent configuration dimension
  nq_ = model->nq;        // configuration dimension
  ny_ = nq_ + nv_ + nc_;  // augmented state dimension
  ndy_ = 2 * nv_ + nc_;   // augmented state tangent space dimension

  // Define internally the limits of the first joint
  const std::size_t nq0 = model->joints[1].nq();
  lb_.head(nq0) = -std::numeric_limits<Scalar>::infinity() * VectorXs::Ones(nq0);
  ub_.head(nq0) = std::numeric_limits<Scalar>::infinity() * VectorXs::Ones(nq0);
  lb_.segment(nq0, nq_ - nq0) = pinocchio_->lowerPositionLimit.tail(nq_ - nq0);
  ub_.segment(nq0, nq_ - nq0) = pinocchio_->upperPositionLimit.tail(nq_ - nq0);
  lb_.segment(nq_, nv_) = -pinocchio_->velocityLimit;
  ub_.segment(nq_, nv_) = pinocchio_->velocityLimit;
  // Visco-elastic force limit (no limit)
  lb_.tail(nc_) = -std::numeric_limits<Scalar>::infinity() * VectorXs::Ones(nc_);
  ub_.tail(nc_) = std::numeric_limits<Scalar>::infinity() * VectorXs::Ones(nc_);

  Base::update_has_limits();

  y0_.head(nq_) = pinocchio::neutral(*pinocchio_.get());
  y0_.tail(nv_ + nc_) = VectorXs::Zero(nv_ + nc_);
  // in order to know y0 we need to know f0 , i.e. oPc ?
  // no, it's measured! Set 0 force. Force as state equivalent to dP as state
}

template <typename Scalar>
StateSoftContactTpl<Scalar>::~StateSoftContactTpl() {}

template <typename Scalar>
const std::size_t& StateSoftContactTpl<Scalar>::get_nc() const {
  return nc_;
}

template <typename Scalar>
const std::size_t& StateSoftContactTpl<Scalar>::get_ny() const {
  return ny_;
}

template <typename Scalar>
const std::size_t& StateSoftContactTpl<Scalar>::get_ndy() const {
  return ndy_;
}

template <typename Scalar>
typename MathBaseTpl<Scalar>::VectorXs StateSoftContactTpl<Scalar>::zero() const {
  return y0_;
}

template <typename Scalar>
typename MathBaseTpl<Scalar>::VectorXs StateSoftContactTpl<Scalar>::rand() const {
  VectorXs yrand = VectorXs::Random(ny_);
  yrand.head(nq_) = pinocchio::randomConfiguration(*pinocchio_.get());
  return yrand;
}

template <typename Scalar>
void StateSoftContactTpl<Scalar>::diff(const Eigen::Ref<const VectorXs>& y0,
                               const Eigen::Ref<const VectorXs>& y1,
                               Eigen::Ref<VectorXs> dyout) const {
  if (static_cast<std::size_t>(y0.size()) != ny_) {
    throw_pretty("Invalid argument: "
                 << "y0 has wrong dimension (it should be " +
                        std::to_string(ny_) + ")");
  }
  if (static_cast<std::size_t>(y1.size()) != ny_) {
    throw_pretty("Invalid argument: "
                 << "y1 has wrong dimension (it should be " +
                        std::to_string(ny_) + ")");
  }
  if (static_cast<std::size_t>(dyout.size()) != ndy_) {
    throw_pretty("Invalid argument: "
                 << "dyout has wrong dimension (it should be " +
                        std::to_string(ndy_) + ")");
  }

  pinocchio::difference(*pinocchio_.get(), y0.head(nq_), y1.head(nq_),
                        dyout.head(nv_));
  dyout.segment(nv_, nv_) = y1.segment(nq_, nv_) - y0.segment(nq_, nv_);
  dyout.tail(nc_) = y1.tail(nc_) - y0.tail(nc_);
}

template <typename Scalar>
void StateSoftContactTpl<Scalar>::integrate(const Eigen::Ref<const VectorXs>& y,
                                    const Eigen::Ref<const VectorXs>& dy,
                                    Eigen::Ref<VectorXs> yout) const {
  if (static_cast<std::size_t>(y.size()) != ny_) {
    throw_pretty("Invalid argument: "
                 << "y has wrong dimension (it should be " +
                        std::to_string(ny_) + ")");
  }
  if (static_cast<std::size_t>(dy.size()) != ndy_) {
    throw_pretty("Invalid argument: "
                 << "dy has wrong dimension (it should be " +
                        std::to_string(ndy_) + ")");
  }
  if (static_cast<std::size_t>(yout.size()) != ny_) {
    throw_pretty("Invalid argument: "
                 << "yout has wrong dimension (it should be " +
                        std::to_string(ny_) + ")");
  }

  pinocchio::integrate(*pinocchio_.get(), y.head(nq_), dy.head(nv_),
                       yout.head(nq_));
  yout.segment(nq_, nv_) = y.segment(nq_, nv_) + dy.segment(nv_, nv_);
  yout.tail(nc_) = y.tail(nc_) + dy.tail(nc_);
}

template <typename Scalar>
void StateSoftContactTpl<Scalar>::Jdiff(const Eigen::Ref<const VectorXs>& y0,
                                const Eigen::Ref<const VectorXs>& y1,
                                Eigen::Ref<MatrixXs> Jfirst,
                                Eigen::Ref<MatrixXs> Jsecond,
                                const Jcomponent firstsecond) const {
  assert_pretty(
      is_a_Jcomponent(firstsecond),
      ("firstsecond must be one of the Jcomponent {both, first, second}"));
  if (static_cast<std::size_t>(y0.size()) != ny_) {
    throw_pretty("Invalid argument: "
                 << "y0 has wrong dimension (it should be " +
                        std::to_string(ny_) + ")");
  }
  if (static_cast<std::size_t>(y1.size()) != ny_) {
    throw_pretty("Invalid argument: "
                 << "y1 has wrong dimension (it should be " +
                        std::to_string(ny_) + ")");
  }

  if (firstsecond == first) {
    if (static_cast<std::size_t>(Jfirst.rows()) != ndy_ ||
        static_cast<std::size_t>(Jfirst.cols()) != ndy_) {
      throw_pretty("Invalid argument: "
                   << "Jfirst has wrong dimension (it should be " +
                          std::to_string(ndy_) + "," + std::to_string(ndy_) +
                          ")");
    }

    pinocchio::dDifference(*pinocchio_.get(), y0.head(nq_), y1.head(nq_),
                           Jfirst.topLeftCorner(nv_, nv_), pinocchio::ARG0);
    Jfirst.block(nv_, nv_, nv_, nv_).diagonal().array() = (Scalar)-1;
    Jfirst.bottomRightCorner(nc_, nc_).diagonal().array() = (Scalar)-1;
  } else if (firstsecond == second) {
    if (static_cast<std::size_t>(Jsecond.rows()) != ndy_ ||
        static_cast<std::size_t>(Jsecond.cols()) != ndy_) {
      throw_pretty("Invalid argument: "
                   << "Jsecond has wrong dimension (it should be " +
                          std::to_string(ndy_) + "," + std::to_string(ndy_) +
                          ")");
    }
    pinocchio::dDifference(*pinocchio_.get(), y0.head(nq_), y1.head(nq_),
                           Jsecond.topLeftCorner(nv_, nv_), pinocchio::ARG1);
    Jsecond.block(nv_, nv_, nv_, nv_).diagonal().array() = (Scalar)1;
    Jsecond.bottomRightCorner(nc_, nc_).diagonal().array() = (Scalar)1;
  } else {  // computing both
    if (static_cast<std::size_t>(Jfirst.rows()) != ndy_ ||
        static_cast<std::size_t>(Jfirst.cols()) != ndy_) {
      throw_pretty("Invalid argument: "
                   << "Jfirst has wrong dimension (it should be " +
                          std::to_string(ndy_) + "," + std::to_string(ndy_) +
                          ")");
    }
    if (static_cast<std::size_t>(Jsecond.rows()) != ndy_ ||
        static_cast<std::size_t>(Jsecond.cols()) != ndy_) {
      throw_pretty("Invalid argument: "
                   << "Jsecond has wrong dimension (it should be " +
                          std::to_string(ndy_) + "," + std::to_string(ndy_) +
                          ")");
    }
    pinocchio::dDifference(*pinocchio_.get(), y0.head(nq_), y1.head(nq_),
                           Jfirst.topLeftCorner(nv_, nv_), pinocchio::ARG0);
    pinocchio::dDifference(*pinocchio_.get(), y0.head(nq_), y1.head(nq_),
                           Jsecond.topLeftCorner(nv_, nv_), pinocchio::ARG1);
    Jfirst.block(nv_, nv_, nv_, nv_).diagonal().array() = (Scalar)-1;
    Jfirst.bottomRightCorner(nc_, nc_).diagonal().array() = (Scalar)-1;
    Jsecond.block(nv_, nv_, nv_, nv_).diagonal().array() = (Scalar)1;
    Jsecond.bottomRightCorner(nc_, nc_).diagonal().array() = (Scalar)1;
  }
}

template <typename Scalar>
void StateSoftContactTpl<Scalar>::Jintegrate(const Eigen::Ref<const VectorXs>& y,
                                     const Eigen::Ref<const VectorXs>& dy,
                                     Eigen::Ref<MatrixXs> Jfirst,
                                     Eigen::Ref<MatrixXs> Jsecond,
                                     const Jcomponent firstsecond,
                                     const AssignmentOp op) const {
  assert_pretty(
      is_a_Jcomponent(firstsecond),
      ("firstsecond must be one of the Jcomponent {both, first, second}"));
  assert_pretty(is_a_AssignmentOp(op),
                ("op must be one of the AssignmentOp {settop, addto, rmfrom}"));
  if (firstsecond == first || firstsecond == both) {
    if (static_cast<std::size_t>(Jfirst.rows()) != ndy_ ||
        static_cast<std::size_t>(Jfirst.cols()) != ndy_) {
      throw_pretty("Invalid argument: "
                   << "Jfirst has wrong dimension (it should be " +
                          std::to_string(ndy_) + "," + std::to_string(ndy_) +
                          ")");
    }
    switch (op) {
      case setto:
        pinocchio::dIntegrate(*pinocchio_.get(), y.head(nq_), dy.head(nv_),
                              Jfirst.topLeftCorner(nv_, nv_), pinocchio::ARG0,
                              pinocchio::SETTO);
        Jfirst.block(nv_, nv_, nv_, nv_).diagonal().array() = (Scalar)1;
        Jfirst.bottomRightCorner(nc_, nc_).diagonal().array() = (Scalar)1;
        break;
      case addto:
        pinocchio::dIntegrate(*pinocchio_.get(), y.head(nq_), dy.head(nv_),
                              Jfirst.topLeftCorner(nv_, nv_), pinocchio::ARG0,
                              pinocchio::ADDTO);
        Jfirst.block(nv_, nv_, nv_, nv_).diagonal().array() += (Scalar)1;
        Jfirst.bottomRightCorner(nc_, nc_).diagonal().array() += (Scalar)1;
        break;
      case rmfrom:
        pinocchio::dIntegrate(*pinocchio_.get(), y.head(nq_), dy.head(nv_),
                              Jfirst.topLeftCorner(nv_, nv_), pinocchio::ARG0,
                              pinocchio::RMTO);
        Jfirst.block(nv_, nv_, nv_, nv_).diagonal().array() -= (Scalar)1;
        Jfirst.bottomRightCorner(nc_, nc_).diagonal().array() -= (Scalar)1;
        break;
      default:
        throw_pretty(
            "Invalid argument: allowed operators: setto, addto, rmfrom");
        break;
    }
  }
  if (firstsecond == second || firstsecond == both) {
    if (static_cast<std::size_t>(Jsecond.rows()) != ndy_ ||
        static_cast<std::size_t>(Jsecond.cols()) != ndy_) {
      throw_pretty("Invalid argument: "
                   << "Jsecond has wrong dimension (it should be " +
                          std::to_string(ndy_) + "," + std::to_string(ndy_) +
                          ")");
    }
    switch (op) {
      case setto:
        pinocchio::dIntegrate(*pinocchio_.get(), y.head(nq_), dy.head(nv_),
                              Jsecond.topLeftCorner(nv_, nv_), pinocchio::ARG1,
                              pinocchio::SETTO);
        Jsecond.block(nv_, nv_, nv_, nv_).diagonal().array() = (Scalar)1;
        Jsecond.bottomRightCorner(nc_, nc_).diagonal().array() = (Scalar)1;
        break;
      case addto:
        pinocchio::dIntegrate(*pinocchio_.get(), y.head(nq_), dy.head(nv_),
                              Jsecond.topLeftCorner(nv_, nv_), pinocchio::ARG1,
                              pinocchio::ADDTO);
        Jsecond.block(nv_, nv_, nv_, nv_).diagonal().array() += (Scalar)1;
        Jsecond.bottomRightCorner(nc_, nc_).diagonal().array() += (Scalar)1;
        break;
      case rmfrom:
        pinocchio::dIntegrate(*pinocchio_.get(), y.head(nq_), dy.head(nv_),
                              Jsecond.topLeftCorner(nv_, nv_), pinocchio::ARG1,
                              pinocchio::RMTO);
        Jsecond.block(nv_, nv_, nv_, nv_).diagonal().array() -= (Scalar)1;
        Jsecond.bottomRightCorner(nc_, nc_).diagonal().array() -= (Scalar)1;
        break;
      default:
        throw_pretty(
            "Invalid argument: allowed operators: setto, addto, rmfrom");
        break;
    }
  }
}

template <typename Scalar>
void StateSoftContactTpl<Scalar>::JintegrateTransport(
    const Eigen::Ref<const VectorXs>& y, const Eigen::Ref<const VectorXs>& dy,
    Eigen::Ref<MatrixXs> Jin, const Jcomponent firstsecond) const {
  assert_pretty(
      is_a_Jcomponent(firstsecond),
      ("firstsecond must be one of the Jcomponent {both, first, second}"));

  switch (firstsecond) {
    case first:
      pinocchio::dIntegrateTransport(*pinocchio_.get(), y.head(nq_),
                                     dy.head(nv_), Jin.topRows(nv_),
                                     pinocchio::ARG0);
      break;
    case second:
      pinocchio::dIntegrateTransport(*pinocchio_.get(), y.head(nq_),
                                     dy.head(nv_), Jin.topRows(nv_),
                                     pinocchio::ARG1);
      break;
    default:
      throw_pretty(
          "Invalid argument: firstsecond must be either first or second. both "
          "not supported for this operation.");
      break;
  }
}

template <typename Scalar>
const boost::shared_ptr<pinocchio::ModelTpl<Scalar> >&
StateSoftContactTpl<Scalar>::get_pinocchio() const {
  return pinocchio_;
}

}  // namespace sobec
