///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2022, LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "sobec/walk-without-think/mpc.hpp"

#include <crocoddyl/core/solvers/fddp.hpp>
#include <eigenpy/eigenpy.hpp>
#include <pinocchio/multibody/fwd.hpp>  // Must be included first!

#include "sobec/fwd.hpp"

namespace sobec {
namespace python {

using namespace crocoddyl;
namespace bp = boost::python;

void exposeMPCParams() {
  bp::register_ptr_to_python<boost::shared_ptr<MPCWalkParams> >();

  bp::class_<MPCWalkParams>(
      "MPCWalkParams",
      bp::init<>(bp::args("self"), "Empty initialization of the MPC params"))
      .def("readFromYaml", &MPCWalkParams::readParamsFromYamlFile,
           bp::args("filename"))
      .add_property("DT", bp::make_getter(&MPCWalkParams::DT),
                    bp::make_setter(&MPCWalkParams::DT), "DT.")
      .add_property("vcomRef", bp::make_getter(&MPCWalkParams::vcomRef),
                    bp::make_setter(&MPCWalkParams::vcomRef), "vcomRef.")
      .add_property("Tmpc", bp::make_getter(&MPCWalkParams::Tmpc),
                    bp::make_setter(&MPCWalkParams::Tmpc), "Tmpc.")
      .add_property("Tstart", bp::make_getter(&MPCWalkParams::Tstart),
                    bp::make_setter(&MPCWalkParams::Tstart), "Tstart.")
      .add_property("Tsingle", bp::make_getter(&MPCWalkParams::Tsingle),
                    bp::make_setter(&MPCWalkParams::Tsingle), "Tsingle.")
      .add_property("Tdouble", bp::make_getter(&MPCWalkParams::Tdouble),
                    bp::make_setter(&MPCWalkParams::Tdouble), "Tdouble.")
      .add_property("Tend", bp::make_getter(&MPCWalkParams::Tend),
                    bp::make_setter(&MPCWalkParams::Tend), "Tend.")
      .add_property(
          "solver_th_stop", bp::make_getter(&MPCWalkParams::solver_th_stop),
          bp::make_setter(&MPCWalkParams::solver_th_stop), "solver_th_stop.")
      .add_property(
          "solver_reg_min", bp::make_getter(&MPCWalkParams::solver_reg_min),
          bp::make_setter(&MPCWalkParams::solver_reg_min), "solver_reg_min.")
      .add_property(
          "solver_maxiter", bp::make_getter(&MPCWalkParams::solver_maxiter),
          bp::make_setter(&MPCWalkParams::solver_maxiter), "solver_maxiter.")

      .add_property("x0", bp::make_getter(&MPCWalkParams::x0),
                    bp::make_setter(&MPCWalkParams::x0), "x0.")
      .add_property("DT", bp::make_getter(&MPCWalkParams::DT),
                    bp::make_setter(&MPCWalkParams::DT), "DT.")

      ;
}

void exposeMPCWalkclass() {
  bp::register_ptr_to_python<boost::shared_ptr<MPCWalk> >();

  bp::class_<MPCWalk>("MPCWalk", bp::init<boost::shared_ptr<MPCWalkParams>,
                                          boost::shared_ptr<ShootingProblem> >(
                                     bp::args("self", "params", "problem"),
                                     "Initialize the MPC (empty init)"))
      .def<void (MPCWalk::*)(const Eigen::Ref<const Eigen::VectorXd>&,
                             const int)>("calc", &MPCWalk::calc,
                                         bp::args("self", "x", "t"))
      .def("initialize", &MPCWalk::initialize)
      // .add_property("Tmpc", &MPCWalk::get_Tmpc, &MPCWalk::set_Tmpc,
      //               "duration of MPC horizon")
      // .add_property("Tstart", &MPCWalk::get_Tstart, &MPCWalk::set_Tstart,
      //               "duration of the starting phase")
      // .add_property("Tdouble", &MPCWalk::get_Tdouble, &MPCWalk::set_Tdouble,
      //               "duration of the double-support phase")
      // .add_property("Tsingle", &MPCWalk::get_Tsingle, &MPCWalk::set_Tsingle,
      //               "duration of the single-support phase")
      // .add_property("Tend", &MPCWalk::get_Tend, &MPCWalk::set_Tend,
      //               "duration of end the phase")
      // .add_property(
      //     "vcomRef",
      //     bp::make_getter(&MPCWalk::vcomRef,
      //     bp::return_internal_reference<>()),
      //     bp::make_setter(&MPCWalk::vcomRef),
      //     "Reference of the com velocity, to tune the MPC at runtime.")
      // .add_property("solver_th_stop",
      // bp::make_getter(&MPCWalk::solver_th_stop),
      //               bp::make_setter(&MPCWalk::solver_th_stop),
      //               "Stop threshold to configure the solver.")
      // .add_property(
      //     "solver_reg_min", bp::make_getter(&MPCWalk::solver_reg_min),
      //     bp::make_setter(&MPCWalk::solver_reg_min),
      //     "reg_min param (minimal regularization) to configure the solver.")
      // .add_property("solver_maxiter",
      // bp::make_getter(&MPCWalk::solver_maxiter),
      //               bp::make_setter(&MPCWalk::solver_maxiter),
      //               "maxiter param to configure the solver.")
      // .add_property("DT", bp::make_getter(&MPCWalk::DT),
      //               bp::make_setter(&MPCWalk::DT),
      //               "time step duration of the shooting nodes.")
      // .add_property(
      //     "x0",
      //     bp::make_getter(&MPCWalk::x0, bp::return_internal_reference<>()),
      //     bp::make_setter(&MPCWalk::x0),
      //     "Reference of the com velocity, to tune the MPC at runtime.")
      .add_property(
          "storage",
          bp::make_getter(&MPCWalk::storage,
                          bp::return_value_policy<bp::return_by_value>()),
          "Shooting storage used for MPC solver")
      .add_property(
          "problem",
          bp::make_getter(&MPCWalk::problem,
                          bp::return_value_policy<bp::return_by_value>()),
          "Shooting problem used for MPC solver")
      .add_property(
          "state",
          bp::make_getter(&MPCWalk::state,
                          bp::return_value_policy<bp::return_by_value>()),
          "State model of the terminal node")
      .add_property(
          "solver",
          bp::make_getter(&MPCWalk::solver,
                          bp::return_value_policy<bp::return_by_value>()),
          "OCP Solver inside the MPC.")

      ;
}

void exposeMPCWalk() {
  exposeMPCParams();
  exposeMPCWalkclass();
}

}  // namespace python
}  // namespace sobec
