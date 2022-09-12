///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2021, LAAS-CNRS, University of Edinburgh, CTU, INRIA,
// University of Oxford Copyright note valid unless otherwise stated in
// individual files. All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "sobec/crocomplements/softcontact/dam1d-augmented.hpp"

#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <eigenpy/eigenpy.hpp>

namespace sobec {
namespace python {
namespace bp = boost::python;

void exposeDAMSoftContact1DAugmentedFwdDyn() {

  bp::register_ptr_to_python<boost::shared_ptr<DAMSoftContact1DAugmentedFwdDynamics>>();

  bp::class_<DAMSoftContact1DAugmentedFwdDynamics, bp::bases<crocoddyl::DifferentialActionModelFreeFwdDynamics>>(
      "DAMSoftContact1DAugmentedFwdDynamics", 
      "Differential action model for visco-elastic contact forward dynamics in multibody systems.",
      bp::init<boost::shared_ptr<crocoddyl::StateMultibody>,
               boost::shared_ptr<crocoddyl::ActuationModelAbstract>,
               boost::shared_ptr<crocoddyl::CostModelSum>,
               pinocchio::FrameIndex, double, double, Eigen::Vector3d, pinocchio::ReferenceFrame, sobec::Vector3MaskType>(
          bp::args("self", "state", "actuation", "costs", "frameId", "Kp", "Kv", "oPc", "ref", "type"),
          "Initialize the constrained forward-dynamics action model.\n\n"
          ":param state: multibody state\n"
          ":param actuation: actuation model\n"
          ":param costs: stack of cost functions\n"
          ":param frameId: Frame id of the contact model\n"
          ":param Kp: Stiffness of the visco-elastic contact model\n"
          ":param Kv: Damping of the visco-elastic contact model\n"
          ":param oPc: Anchor point of the contact model\n"
          ":param ref: Pinocchio reference frame of the contact\n"
          ":param type: Contact 1D mask type "))
      .def<void (DAMSoftContact1DAugmentedFwdDynamics::*)(
          const boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&,
          const double&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &DAMSoftContact1DAugmentedFwdDynamics::calc,
          bp::args("self", "data", "x", "f", "u"),
          "Compute the next state and cost value.\n\n"
          "It describes the time-continuous evolution of the multibody system under a visco-elastic contact.\n"
          "Additionally it computes the cost value associated to this state and control pair.\n"
          ":param data: soft contact 1d forward-dynamics action data\n"
          ":param x: continuous-time state vector\n"
          ":param f: continuous-time force vector\n"
          ":param u: continuous-time control input")
      .def<void (DAMSoftContact1DAugmentedFwdDynamics::*)(
          const boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&, 
          const double&)>(
          "calc", &DAMSoftContact1DAugmentedFwdDynamics::calc, bp::args("self", "data", "x", "f"))
      
      .def<void (DAMSoftContact1DAugmentedFwdDynamics::*)(
          const boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&,
          const double&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &DAMSoftContact1DAugmentedFwdDynamics::calcDiff,
          bp::args("self", "data", "x", "f", "u"),
          "Compute the derivatives of the differential multibody system and\n"
          "its cost functions.\n\n"
          "It computes the partial derivatives of the differential multibody system and the\n"
          "cost function. It assumes that calc has been run first.\n"
          "This function builds a quadratic approximation of the\n"
          "action model (i.e. dynamical system and cost function).\n"
          ":param data: soft contact 1d differential forward-dynamics action data\n"
          ":param x: time-continuous state vector\n"
          ":param x: time-continuous force vector\n"
          ":param u: time-continuous control input\n")
      .def<void (DAMSoftContact1DAugmentedFwdDynamics::*)(
          const boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract>&, 
          const Eigen::Ref<const Eigen::VectorXd>&, 
          const double&)>(
          "calcDiff", &DAMSoftContact1DAugmentedFwdDynamics::calcDiff, bp::args("self", "data", "x", "f"))
      .def("createData", &DAMSoftContact1DAugmentedFwdDynamics::createData,
           bp::args("self"), "Create the Euler integrator data.")
      .def("set_force_cost", &DAMSoftContact1DAugmentedFwdDynamics::set_force_cost,
           bp::args("self", "force_des", "force_weight"),
           "Initialize force reference and cost weight ")
      .add_property(
          "Kp",
          bp::make_function(&DAMSoftContact1DAugmentedFwdDynamics::get_Kp,
                            bp::return_value_policy<bp::return_by_value>()),
          &DAMSoftContact1DAugmentedFwdDynamics::set_Kp,
          "Stiffness of the contact model")
      .add_property(
          "Kv",
          bp::make_function(&DAMSoftContact1DAugmentedFwdDynamics::get_Kv,
                            bp::return_value_policy<bp::return_by_value>()),
          &DAMSoftContact1DAugmentedFwdDynamics::set_Kv,
          "Damping of the contact model")
      .add_property(
          "oPc",
          bp::make_function(&DAMSoftContact1DAugmentedFwdDynamics::get_oPc,
                            bp::return_value_policy<bp::return_by_value>()),
          &DAMSoftContact1DAugmentedFwdDynamics::set_oPc,
          "Anchor point of the contact model")
      .add_property(
          "f_des",
          bp::make_function(&DAMSoftContact1DAugmentedFwdDynamics::get_force_des,
                            bp::return_value_policy<bp::return_by_value>()),
          &DAMSoftContact1DAugmentedFwdDynamics::set_force_des,
          "Desired force in the cost")
      .add_property(
          "f_weight",
          bp::make_function(&DAMSoftContact1DAugmentedFwdDynamics::get_force_weight,
                            bp::return_value_policy<bp::return_by_value>()),
          &DAMSoftContact1DAugmentedFwdDynamics::set_force_weight,
          "Force cost weight")
      .add_property(
          "ref",
          bp::make_function(&DAMSoftContact1DAugmentedFwdDynamics::get_ref,
                            bp::return_value_policy<bp::return_by_value>()),
          &DAMSoftContact1DAugmentedFwdDynamics::set_ref,
          "Pinocchio reference frame")
      .add_property(
          "id",
          bp::make_function(&DAMSoftContact1DAugmentedFwdDynamics::get_id,
                            bp::return_value_policy<bp::return_by_value>()),
          &DAMSoftContact1DAugmentedFwdDynamics::set_id,
          "Contact frame id")
      .add_property(
          "armature",
          bp::make_function(&DAMSoftContact1DAugmentedFwdDynamics::get_armature,
                            bp::return_value_policy<bp::return_by_value>()),
          &DAMSoftContact1DAugmentedFwdDynamics::set_armature,
          "Armature")
      .add_property(
          "type",
          bp::make_function(&DAMSoftContact1DAugmentedFwdDynamics::get_type,
                            bp::return_value_policy<bp::return_by_value>()),
          &DAMSoftContact1DAugmentedFwdDynamics::set_type,
          "1D mask type");

  bp::register_ptr_to_python<boost::shared_ptr<DADSoftContact1DAugmentedFwdDynamics> >();

  bp::class_<DADSoftContact1DAugmentedFwdDynamics, bp::bases<crocoddyl::DifferentialActionDataFreeFwdDynamics> >(
      "DADSoftContact1DAugmentedFwdDynamics", "Action data for the soft contact 1D forward dynamics system",
      bp::init<DAMSoftContact1DAugmentedFwdDynamics*>(
          bp::args("self", "model"),
          "Create soft contact 1D forward-dynamics action data.\n\n"
          ":param model: soft contact 1D model"))
      .add_property(
          "lJ",
          bp::make_getter(&sobec::DADSoftContact1DAugmentedFwdDynamics::lJ,
                          bp::return_internal_reference<>()),
          "Jacobian of the contact frame in LOCAL")
      .add_property(
          "oJ",
          bp::make_getter(&sobec::DADSoftContact1DAugmentedFwdDynamics::oJ,
                          bp::return_internal_reference<>()),
          "Jacobian of the contact frame in LOCAL_WORLD_ALIGNED");
    //   .add_property(
    //       "lv_partial_dq",
    //       bp::make_getter(&sobec::DADSoftContact1DAugmentedFwdDynamics::lv_partial_dq,
    //                       bp::return_internal_reference<>()),
    //       "Partial derivative of LOCAL contact frame velocity w.r.t. joint positions")
    //   .add_property(
    //       "lv_partial_dv",
    //       bp::make_getter(&sobec::DADSoftContact1DAugmentedFwdDynamics::lv_partial_dv,
    //                       bp::return_internal_reference<>()),
    //       "Partial derivative of LOCAL contact frame velocity w.r.t. joint velocities")
    //   .add_property(
    //       "aba_dq",
    //       bp::make_getter(&sobec::DADSoftContact1DAugmentedFwdDynamics::aba_dq,
    //                       bp::return_internal_reference<>()),
    //       "Partial derivative of joint acceleration w.r.t. joint positions")
    //   .add_property(
    //       "aba_dv",
    //       bp::make_getter(&sobec::DADSoftContact1DAugmentedFwdDynamics::aba_dv,
    //                       bp::return_internal_reference<>()),
    //       "Partial derivative of joint accelerations w.r.t. joint velocities")
    //   .add_property(
    //       "aba_dtau",
    //       bp::make_getter(&sobec::DADSoftContact1DAugmentedFwdDynamics::aba_dtau,
    //                       bp::return_internal_reference<>()),
    //       "Partial derivative of joint accelerations w.r.t. joint torques")
    //   .add_property(
    //       "df_dx",
    //       bp::make_getter(&sobec::DADSoftContact1DAugmentedFwdDynamics::df_dx,
    //                       bp::return_internal_reference<>()),
    //       "Partial derivative of contact force w.r.t. state")
    //   .add_property(
    //       "df_dx_copy",
    //       bp::make_getter(&sobec::DADSoftContact1DAugmentedFwdDynamics::df_dx_copy,
    //                       bp::return_internal_reference<>()),
    //       "Partial derivative of contact force w.r.t. state (copy)")
    //   .add_property(
    //       "pinForce",
    //       bp::make_getter(&sobec::DADSoftContact1DAugmentedFwdDynamics::pinForce,
    //                       bp::return_internal_reference<>()),
    //       "Spatial wrench due to visco-elastic contact, expressed in LOCAL frame coordinates")
    //   .add_property(
    //       "fext",
    //       bp::make_getter(&sobec::DADSoftContact1DAugmentedFwdDynamics::fext,
    //                       bp::return_internal_reference<>()),
    //       "Vector of spatial wrenches due to visco-elastic contact, expressed in LOCAL joint coordinates")
    //   .add_property(
    //       "fext_copy",
    //       bp::make_getter(&sobec::DADSoftContact1DAugmentedFwdDynamics::fext_copy,
    //                       bp::return_internal_reference<>()),
    //       "Vector of spatial wrenches due to visco-elastic contact, expressed in LOCAL joint coordinates (copy)");
}

}  // namespace python
}  // namespace sobec
