///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2021, LAAS-CNRS, University of Edinburgh, CTU, INRIA,
// University of Oxford Copyright note valid unless otherwise stated in
// individual files. All rights reserved.
///////////////////////////////////////////////////////////////////////////////

// #include "sobec/crocomplements/softcontact/dam-augmented.hpp"
#include "dam-augmented.hpp"

#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <eigenpy/eigenpy.hpp>

namespace sobec {
namespace python {
namespace bp = boost::python;

void exposeDAMSoftContactAbstractAugmentedFwdDyn() {

  bp::register_ptr_to_python<boost::shared_ptr<DAMSoftContactAbstractAugmentedFwdDynamics>>();

  bp::class_<DAMSoftContactAbstractAugmentedFwdDynamics, bp::bases<crocoddyl::DifferentialActionModelFreeFwdDynamics>>(
  // bp::class_<DAMSoftContactAbstractAugmentedFwdDynamics, boost::noncopyable>(
      "DAMSoftContactAbstractAugmentedFwdDynamics", 
      "Differential action model for visco-elastic contact forward dynamics in multibody systems.",
      bp::init<boost::shared_ptr<crocoddyl::StateMultibody>,
               boost::shared_ptr<crocoddyl::ActuationModelAbstract>,
               boost::shared_ptr<crocoddyl::CostModelSum>,
               pinocchio::FrameIndex, double, double, Eigen::Vector3d, int, bp::optional<pinocchio::ReferenceFrame>>(
          bp::args("self", "state", "actuation", "costs", "frameId", "Kp", "Kv", "oPc", "nc", "ref"),
          "Initialize the constrained forward-dynamics action model.\n\n"
          ":param state: multibody state\n"
          ":param actuation: actuation model\n"
          ":param costs: stack of cost functions\n"
          ":param frameId: Frame id of the contact model\n"
          ":param Kp: Stiffness of the visco-elastic contact model\n"
          ":param Kv: Damping of the visco-elastic contact model\n"
          ":param oPc: Anchor point of the contact model\n"
          ":param nc: Dimension of the contact model\n"
          ":param ref: Pinocchio reference frame of the contact"))
      // .def("calc", bp::pure_virtual(&DAMSoftContactAbstractAugmentedFwdDynamics_wrap::calc), bp::args("self", "data", "x", "f", "u"),
      //      "Compute the system acceleration and cost value.\n\n"
      //      ":param data: differential action data\n"
      //      ":param x: state point (dim. state.nx)\n"
      //      ":param f: force point (dim. state.nx)\n"
      //      ":param u: control input (dim. nu)")
      // .def("calc", bp::pure_virtual(&DAMSoftContactAbstractAugmentedFwdDynamics_wrap::calc), bp::args("self", "data", "x", "f"),
      //      "Compute the system acceleration and cost value.\n\n"
      //      ":param data: differential action data\n"
      //      ":param x: state point (dim. state.nx)\n"
      //      ":param f: force point (dim. state.nx)")
      .def<void (DAMSoftContactAbstractAugmentedFwdDynamics::*)(
          const boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&,
          const Eigen::Ref<const Eigen::VectorXd>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &DAMSoftContactAbstractAugmentedFwdDynamics::calc,
          bp::args("self", "data", "x", "f", "u"),
          "Compute the next state and cost value.\n\n"
          "It describes the time-continuous evolution of the multibody system under a visco-elastic contact.\n"
          "Additionally it computes the cost value associated to this state and control pair.\n"
          ":param data: soft contact forward-dynamics action data\n"
          ":param x: continuous-time state vector\n"
          ":param f: continuous-time force vector\n"
          ":param u: continuous-time control input")
      .def<void (DAMSoftContactAbstractAugmentedFwdDynamics::*)(
          const boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&, 
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &DAMSoftContactAbstractAugmentedFwdDynamics::calc, bp::args("self", "data", "x", "f"))

      // .def("calcDiff", bp::pure_virtual(&DAMSoftContactAbstractAugmentedFwdDynamics_wrap::calcDiff),
      //      bp::args("self", "data", "x", "f", "u"),
      //      "Compute the derivatives of the dynamics and cost functions.\n\n"
      //      "It computes the partial derivatives of the dynamical system and the cost\n"
      //      "function. It assumes that calc has been run first.\n"
      //      "This function builds a quadratic approximation of the\n"
      //      "time-continuous action model (i.e. dynamical system and cost function).\n"
      //      ":param data: differential action data\n"
      //      ":param x: state point (dim. state.nx)\n"
      //      ":param f: force point (dim. state.nc)\n"
      //      ":param u: control input (dim. nu)")
      // .def("calcDiff", bp::pure_virtual(&DAMSoftContactAbstractAugmentedFwdDynamics_wrap::calcDiff),
      //      bp::args("self", "data", "x", "f"),
      //      "Compute the derivatives of the dynamics and cost functions.\n\n"
      //      "It computes the partial derivatives of the dynamical system and the cost\n"
      //      "function. It assumes that calc has been run first.\n"
      //      "This function builds a quadratic approximation of the\n"
      //      "time-continuous action model (i.e. dynamical system and cost function).\n"
      //      ":param data: differential action data\n"
      //      ":param x: state point (dim. state.nx)\n"
      //      ":param f: force point (dim. state.nc)")

      .def<void (DAMSoftContactAbstractAugmentedFwdDynamics::*)(
          const boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&,
          const Eigen::Ref<const Eigen::VectorXd>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &DAMSoftContactAbstractAugmentedFwdDynamics::calcDiff,
          bp::args("self", "data", "x", "f", "u"),
          "Compute the derivatives of the differential multibody system and\n"
          "its cost functions.\n\n"
          "It computes the partial derivatives of the differential multibody system and the\n"
          "cost function. It assumes that calc has been run first.\n"
          "This function builds a quadratic approximation of the\n"
          "action model (i.e. dynamical system and cost function).\n"
          ":param data: soft contact differential forward-dynamics action data\n"
          ":param x: time-continuous state vector\n"
          ":param x: time-continuous force vector\n"
          ":param u: time-continuous control input\n")
      .def<void (DAMSoftContactAbstractAugmentedFwdDynamics::*)(
          const boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract>&, 
          const Eigen::Ref<const Eigen::VectorXd>&, 
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &DAMSoftContactAbstractAugmentedFwdDynamics::calcDiff, bp::args("self", "data", "x", "f"))

      // .def("createData", &DAMSoftContactAbstractAugmentedFwdDynamics_wrap::createData, bp::args("self"),
      //      "Create the differential action data.\n\n"
      //      "Each differential action model has its own data that needs to be\n"
      //      "allocated. This function returns the allocated data for a predefined\n"
      //      "DAM.\n"
      //      ":return DAM data.");

      .def("createData", &DAMSoftContactAbstractAugmentedFwdDynamics::createData,
           bp::args("self"), "Create the Euler integrator data.")
    
    //   .def("set_force_cost", &DAMSoftContactAbstractAugmentedFwdDynamics::set_force_cost,
    //        bp::args("self", "force_des", "force_weight"),
    //        "Initialize force reference and cost weight ")
      .add_property(
          "Kp",
          bp::make_function(&DAMSoftContactAbstractAugmentedFwdDynamics::get_Kp,
                            bp::return_value_policy<bp::return_by_value>()),
          &DAMSoftContactAbstractAugmentedFwdDynamics::set_Kp,
          "Stiffness of the contact model")
      .add_property(
          "Kv",
          bp::make_function(&DAMSoftContactAbstractAugmentedFwdDynamics::get_Kv,
                            bp::return_value_policy<bp::return_by_value>()),
          &DAMSoftContactAbstractAugmentedFwdDynamics::set_Kv,
          "Damping of the contact model")
      .add_property(
          "oPc",
          bp::make_function(&DAMSoftContactAbstractAugmentedFwdDynamics::get_oPc,
                            bp::return_value_policy<bp::return_by_value>()),
          &DAMSoftContactAbstractAugmentedFwdDynamics::set_oPc,
          "Anchor point of the contact model")
    // //   .add_property(
    // //       "f_des",
    // //       bp::make_function(&DAMSoftContactAbstractAugmentedFwdDynamics::get_force_des,
    // //                         bp::return_value_policy<bp::return_by_value>()),
    // //       &DAMSoftContactAbstractAugmentedFwdDynamics::set_force_des,
    // //       "Desired force in the cost")
    // //   .add_property(
    // //       "f_weight",
    // //       bp::make_function(&DAMSoftContactAbstractAugmentedFwdDynamics::get_force_weight,
    // //                         bp::return_value_policy<bp::return_by_value>()),
    // //       &DAMSoftContactAbstractAugmentedFwdDynamics::set_force_weight,
    // //       "Force cost weight")
      .add_property(
          "ref",
          bp::make_function(&DAMSoftContactAbstractAugmentedFwdDynamics_wrap::get_ref,
                            bp::return_value_policy<bp::return_by_value>()),
          &DAMSoftContactAbstractAugmentedFwdDynamics::set_ref,
          "Pinocchio reference frame")
      .add_property(
          "id",
          bp::make_function(&DAMSoftContactAbstractAugmentedFwdDynamics_wrap::get_id,
                            bp::return_value_policy<bp::return_by_value>()),
          &DAMSoftContactAbstractAugmentedFwdDynamics::set_id,
          "Contact frame id")
      .add_property(
          "armature",
          bp::make_function(&DAMSoftContactAbstractAugmentedFwdDynamics_wrap::get_armature,
                            bp::return_value_policy<bp::return_by_value>()),
          &DAMSoftContactAbstractAugmentedFwdDynamics::set_armature,
          "Armature");

  bp::register_ptr_to_python<boost::shared_ptr<DADSoftContactAbstractAugmentedFwdDynamics> >();

//   bp::class_<DADSoftContactAbstractAugmentedFwdDynamics, bp::bases<crocoddyl::DifferentialActionModelFreeFwdDynamics>, boost::noncopyable>(
  bp::class_<DADSoftContactAbstractAugmentedFwdDynamics>(
      "DADSoftContactAbstractAugmentedFwdDynamics", "Action data for the soft contact forward dynamics system",
      bp::init<DAMSoftContactAbstractAugmentedFwdDynamics*>(
          bp::args("self", "model"),
          "Create soft contact forward-dynamics action data.\n\n"
          ":param model: soft contact model"))
      .add_property(
          "lJ",
          bp::make_getter(&sobec::DADSoftContactAbstractAugmentedFwdDynamics::lJ,
                          bp::return_internal_reference<>()),
          "Jacobian of the contact frame in LOCAL")
      .add_property(
          "oJ",
          bp::make_getter(&sobec::DADSoftContactAbstractAugmentedFwdDynamics::oJ,
                          bp::return_internal_reference<>()),
          "Jacobian of the contact frame in LOCAL_WORLD_ALIGNED")
      .add_property(
          "aba_dq",
          bp::make_getter(&sobec::DADSoftContactAbstractAugmentedFwdDynamics::aba_dq,
                          bp::return_internal_reference<>()),
          "Partial derivative of joint acceleration w.r.t. joint positions")
      .add_property(
          "aba_dv",
          bp::make_getter(&sobec::DADSoftContactAbstractAugmentedFwdDynamics::aba_dv,
                          bp::return_internal_reference<>()),
          "Partial derivative of joint accelerations w.r.t. joint velocities")
      .add_property(
          "aba_dx",
          bp::make_getter(&sobec::DADSoftContactAbstractAugmentedFwdDynamics::aba_dx,
                          bp::return_internal_reference<>()),
          "Partial derivative of joint accelerations w.r.t. state")
      .add_property(
          "aba_dtau",
          bp::make_getter(&sobec::DADSoftContactAbstractAugmentedFwdDynamics::aba_dtau,
                          bp::return_internal_reference<>()),
          "Partial derivative of joint accelerations w.r.t. joint torques")
      .add_property(
          "aba_df",
          bp::make_getter(&sobec::DADSoftContactAbstractAugmentedFwdDynamics::aba_df,
                          bp::return_internal_reference<>()),
          "Partial derivative of joint accelerations w.r.t. contact force")
      .add_property(
          "lv_dq",
          bp::make_getter(&sobec::DADSoftContactAbstractAugmentedFwdDynamics::lv_dq,
                          bp::return_internal_reference<>()),
          "Partial derivative of LOCAL contact frame velocity w.r.t. joint positions")
      .add_property(
          "lv_dv",
          bp::make_getter(&sobec::DADSoftContactAbstractAugmentedFwdDynamics::lv_dv,
                          bp::return_internal_reference<>()),
          "Partial derivative of LOCAL contact frame velocity w.r.t. joint velocities")
      .add_property(
          "lv_dx",
          bp::make_getter(&sobec::DADSoftContactAbstractAugmentedFwdDynamics::lv_dx,
                          bp::return_internal_reference<>()),
          "Partial derivative of LOCAL contact frame velocity w.r.t. state")

      .add_property(
          "a_dq",
          bp::make_getter(&sobec::DADSoftContactAbstractAugmentedFwdDynamics::a_dq,
                          bp::return_internal_reference<>()),
          "Partial derivative of LOCAL contact frame acceleration w.r.t. joint positions")
      .add_property(
          "a_dv",
          bp::make_getter(&sobec::DADSoftContactAbstractAugmentedFwdDynamics::a_dv,
                          bp::return_internal_reference<>()),
          "Partial derivative of LOCAL contact frame acceleration w.r.t. joint velocities")
      .add_property(
          "a_da",
          bp::make_getter(&sobec::DADSoftContactAbstractAugmentedFwdDynamics::a_da,
                          bp::return_internal_reference<>()),
          "Partial derivative of LOCAL contact frame acceleration w.r.t. joint acceleration")
      .add_property(
          "da_dx",
          bp::make_getter(&sobec::DADSoftContactAbstractAugmentedFwdDynamics::da_dx,
                          bp::return_internal_reference<>()),
          "Partial derivative of LOCAL contact frame acceleration w.r.t. state")
      .add_property(
          "da_du",
          bp::make_getter(&sobec::DADSoftContactAbstractAugmentedFwdDynamics::da_du,
                          bp::return_internal_reference<>()),
          "Partial derivative of LOCAL contact frame acceleration w.r.t. joint torques")
      .add_property(
          "da_df",
          bp::make_getter(&sobec::DADSoftContactAbstractAugmentedFwdDynamics::da_df,
                          bp::return_internal_reference<>()),
          "Partial derivative of LOCAL contact frame acceleration w.r.t. contact force");
          // missing dfdt_dx, dfdt_du, fext, pinForce
}

}  // namespace python
}  // namespace sobec
