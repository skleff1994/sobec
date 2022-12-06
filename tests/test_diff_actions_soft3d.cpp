///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2021, LAAS-CNRS, New York University, Max Planck
// Gesellschaft, INRIA, University of Oxford Copyright note valid unless
// otherwise stated in individual files. All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#define BOOST_TEST_NO_MAIN
#define BOOST_TEST_ALTERNATIVE_INIT_API

#include "common.hpp"
#include "factory/diff-action-soft3d.hpp"
#include <crocoddyl/multibody/residuals/control-gravity.hpp>

using namespace boost::unit_test;
using namespace sobec::unittest;

//----------------------------------------------------------------------------//

void test_check_data(DAMSoftContact3DTypes::Type action_type,
                     PinocchioReferenceTypes::Type ref_type) {
  // create the model
  DAMSoftContact3DFactory factory;
  boost::shared_ptr<sobec::DAMSoftContact3DAugmentedFwdDynamics> model =
      factory.create(action_type, ref_type);
  // Run the print function
  std::ostringstream tmp;
  tmp << *model;
  // create the corresponding data object
  boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract> data =
      model->createData();
  BOOST_CHECK(model->checkData(data));
}

void test_calc_returns_state(DAMSoftContact3DTypes::Type action_type,
                             PinocchioReferenceTypes::Type ref_type) {
  // create the model
  DAMSoftContact3DFactory factory;
  boost::shared_ptr<sobec::DAMSoftContact3DAugmentedFwdDynamics> model = factory.create(action_type, ref_type);
  // create the corresponding data object
  boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract> data = model->createData();
  // Generating random state and control vectors
  const Eigen::VectorXd x = model->get_state()->rand();
  const Eigen::VectorXd f = Eigen::Vector3d::Random();
  const Eigen::VectorXd u = Eigen::VectorXd::Random(model->get_nu());
  // Getting the state dimension from calc() call
  model->calc(data, x, f, u);
  BOOST_CHECK(static_cast<std::size_t>(data->xout.size()) == model->get_state()->get_nv());
}

void test_calc_returns_a_cost(DAMSoftContact3DTypes::Type action_type,
                              PinocchioReferenceTypes::Type ref_type) {
  // create the model
  DAMSoftContact3DFactory factory;
  boost::shared_ptr<sobec::DAMSoftContact3DAugmentedFwdDynamics> model =
      factory.create(action_type, ref_type);
  // create the corresponding data object and set the cost to nan
  boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract> data =
      model->createData();
  data->cost = nan("");
  // Getting the cost value computed by calc()
  const Eigen::VectorXd x = model->get_state()->rand();
  const Eigen::VectorXd f = Eigen::Vector3d::Random();
  const Eigen::VectorXd u = Eigen::VectorXd::Random(model->get_nu());
  model->calc(data, x, f, u);
  // Checking that calc returns a cost value
  BOOST_CHECK(!std::isnan(data->cost));
}



// Test partials against numdiff
void test_partials_numdiff(boost::shared_ptr<sobec::DAMSoftContact3DAugmentedFwdDynamics> model){
  // create the corresponding data object and set the cost to nan
  boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract> data = model->createData();
  // Generating random values for the state and control
  std::size_t ndx = model->get_state()->get_ndx();
  std::size_t nx = model->get_state()->get_nx();
  std::size_t nc = model->get_nc();
  std::size_t nu = model->get_nu();
  Eigen::VectorXd x = model->get_state()->rand();
  Eigen::VectorXd f = Eigen::VectorXd::Random(nc);
  Eigen::VectorXd u = Eigen::VectorXd::Random(nu);
  // Computing the action derivatives
  model->calc(data, x, f, u);
  model->calcDiff(data, x, f, u);

  // numdiff by hand because ND not adapted to augmented calc and calcDiff
  const Eigen::VectorXd& xn0 = data->xout;
  const Eigen::VectorXd& fn0 = boost::static_pointer_cast<sobec::DADSoftContact3DAugmentedFwdDynamics>(data)->fout;
  const double c0 = data->cost;
  // perturbations
  Eigen::VectorXd dx = Eigen::VectorXd::Zero(ndx);
  Eigen::VectorXd df = Eigen::VectorXd::Zero(nc);
  Eigen::VectorXd du = Eigen::VectorXd::Zero(nu);
  Eigen::VectorXd xp = Eigen::VectorXd::Zero(nx);
  Eigen::VectorXd fp = Eigen::VectorXd::Zero(nc);
  double disturbance = std::sqrt(2.0 * std::numeric_limits<double>::epsilon());
  // data
  boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract> data_num_diff = model->createData();
  boost::shared_ptr<sobec::DADSoftContact3DAugmentedFwdDynamics> data_num_diff_cast = boost::static_pointer_cast<sobec::DADSoftContact3DAugmentedFwdDynamics>(data_num_diff);
  std::vector<boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract>> data_x;
  std::vector<boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract>> data_f;
  std::vector<boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract>> data_u;
  for (std::size_t i = 0; i < ndx; ++i) {
      data_x.push_back(model->createData());
  }
  for (std::size_t i = 0; i < nc; ++i) {
      data_f.push_back(model->createData());
  }
  for (std::size_t i = 0; i < nu; ++i) {
      data_u.push_back(model->createData());
  }

  // Computing the d action(x,f,u) / dx
  for (std::size_t ix = 0; ix < ndx; ++ix) {
    dx(ix) = disturbance;
    model->get_state()->integrate(x, dx, xp);
    model->calc(data_x[ix], xp, f, u);
    boost::shared_ptr<sobec::DADSoftContact3DAugmentedFwdDynamics> data_ix_cast = boost::static_pointer_cast<sobec::DADSoftContact3DAugmentedFwdDynamics>(data_x[ix]);
    const Eigen::VectorXd& xn = data_ix_cast->xout;
    const Eigen::VectorXd& fn = data_ix_cast->fout;
    const double c = data_ix_cast->cost;
    data_num_diff_cast->Fx.col(ix) = (xn - xn0) / disturbance;
    data_num_diff_cast->dfdt_dx.col(ix) = (fn - fn0) / disturbance;
    data_num_diff_cast->Lx(ix) = (c - c0) / disturbance;
    dx(ix) = 0.0;
  }

  // Computing the d action(x,f,u) / df
  for (std::size_t idf = 0; idf < nc; ++idf) {
    df(idf) = disturbance;
    model->calc(data_f[idf], x, f + df, u);
    boost::shared_ptr<sobec::DADSoftContact3DAugmentedFwdDynamics> data_idf_cast = boost::static_pointer_cast<sobec::DADSoftContact3DAugmentedFwdDynamics>(data_f[idf]);
    const Eigen::VectorXd& xn = data_idf_cast->xout;
    const Eigen::VectorXd& fn = data_idf_cast->fout;
    const double c = data_idf_cast->cost;
    data_num_diff_cast->aba_df.col(idf) = (xn - xn0) / disturbance;
    data_num_diff_cast->dfdt_df.col(idf) = (fn - fn0) / disturbance;
    data_num_diff_cast->Lf(idf) = (c - c0) / disturbance;
    df(idf) = 0.0;
  }

  // Computing the d action(x,f,u) / du
  du.setZero();
  for (unsigned iu = 0; iu < nu; ++iu) {
    du(iu) = disturbance;
    model->calc(data_u[iu], x, f, u + du);
    boost::shared_ptr<sobec::DADSoftContact3DAugmentedFwdDynamics> data_iu_cast = boost::static_pointer_cast<sobec::DADSoftContact3DAugmentedFwdDynamics>(data_u[iu]);
    const Eigen::VectorXd& xn = data_iu_cast->xout;
    const Eigen::VectorXd& fn = data_iu_cast->fout;
    const double c = data_iu_cast->cost;
    data_num_diff_cast->Fu.col(iu) = (xn - xn0) / disturbance;
    data_num_diff_cast->dfdt_du.col(iu) = (fn - fn0) / disturbance;
    data_num_diff_cast->Lu(iu) = (c - c0) / disturbance;
    du(iu) = 0.0;
  }

  // Checking the partial derivatives against NumDiff
  boost::shared_ptr<sobec::DADSoftContact3DAugmentedFwdDynamics> datacast = boost::static_pointer_cast<sobec::DADSoftContact3DAugmentedFwdDynamics>(data);
  double tol = 1e-4; //sqrt(disturbance);
  BOOST_CHECK((datacast->Fx - data_num_diff_cast->Fx).isZero(NUMDIFF_MODIFIER * tol));
  BOOST_CHECK((datacast->Fu - data_num_diff_cast->Fu).isZero(NUMDIFF_MODIFIER * tol));
  BOOST_CHECK((datacast->aba_df - data_num_diff_cast->aba_df).isZero(NUMDIFF_MODIFIER * tol));
// //   BOOST_CHECK((datacast->aba_dtau - data_num_diff_cast->aba_dtau).isZero(NUMDIFF_MODIFIER * tol));
  BOOST_CHECK((datacast->dfdt_dx - data_num_diff_cast->dfdt_dx).isZero(NUMDIFF_MODIFIER * tol));
  BOOST_CHECK((datacast->dfdt_df - data_num_diff_cast->dfdt_df).isZero(NUMDIFF_MODIFIER * tol));
  BOOST_CHECK((datacast->dfdt_du - data_num_diff_cast->dfdt_du).isZero(NUMDIFF_MODIFIER * tol));
  BOOST_CHECK((datacast->Lx - data_num_diff_cast->Lx).isZero(NUMDIFF_MODIFIER * tol));
  BOOST_CHECK((datacast->Lu - data_num_diff_cast->Lu).isZero(NUMDIFF_MODIFIER * tol));
  if(!(datacast->Lf - data_num_diff_cast->Lf).isZero(NUMDIFF_MODIFIER*tol)){
    std::cout << "Lf " << std::endl;
    std::cout << datacast->Lf << std::endl;
    std::cout << "Lf_ND " << std::endl;
    std::cout << data_num_diff_cast->Lf << std::endl;
  }
  BOOST_CHECK((datacast->Lf - data_num_diff_cast->Lf).isZero(NUMDIFF_MODIFIER * tol));
}

void test_partial_derivatives_against_numdiff(
    DAMSoftContact3DTypes::Type action_type,
    PinocchioReferenceTypes::Type ref_type) {
  // create the model
  DAMSoftContact3DFactory factory;
  boost::shared_ptr<sobec::DAMSoftContact3DAugmentedFwdDynamics> model = factory.create(action_type, ref_type);
  test_partials_numdiff(model);
}

void test_partial_derivatives_against_numdiff_armature(
    DAMSoftContact3DTypes::Type action_type,
    PinocchioReferenceTypes::Type ref_type) {
  // create the model
  DAMSoftContact3DFactory factory;
  boost::shared_ptr<sobec::DAMSoftContact3DAugmentedFwdDynamics> model = factory.create(action_type, ref_type);
  Eigen::VectorXd armature = 1e-3*Eigen::VectorXd::Ones(model->get_state()->get_nv());
  if(model->get_state()->get_nv() > model->get_nu()){
    armature.head(6) = Eigen::VectorXd::Zero(6);
  }
  model->set_armature(armature);
  test_partials_numdiff(model);
}




// Test equivalence with free dynamics when Kp,Kv=0
void test_calc_free(boost::shared_ptr<sobec::DAMSoftContact3DAugmentedFwdDynamics> modelsoft,
                    Eigen::VectorXd armature) {
  // Create DAM free
  boost::shared_ptr<crocoddyl::StateMultibody> statemb = boost::static_pointer_cast<crocoddyl::StateMultibody>(modelsoft->get_state()); 
  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> modelfree = boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(
          statemb, modelsoft->get_actuation(), modelsoft->get_costs());
  // Add gravity cost on free model
  boost::shared_ptr<crocoddyl::CostModelAbstract> cost = boost::make_shared<crocoddyl::CostModelResidual>(
          statemb, boost::make_shared<crocoddyl::ResidualModelControlGrav>( statemb, modelfree->get_actuation()->get_nu() ));
  modelfree->get_costs()->addCost( "grav_reg", cost, 0.01);
  // optional armature
  if(!armature.isZero(1e-9)){
    if(modelsoft->get_state()->get_nv() > modelsoft->get_nu()){
      armature.head(6) = Eigen::VectorXd::Zero(6);
    }
    modelfree->set_armature(armature);
    modelsoft->set_armature(armature);
  }
  boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract> datafree = modelfree->createData();
  boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract> datasoft = modelsoft->createData();
  // Generating random state and control vectors
  const Eigen::VectorXd x = modelsoft->get_state()->rand();
  const Eigen::VectorXd f = Eigen::Vector3d::Zero();
  const Eigen::VectorXd u = Eigen::VectorXd::Random(modelsoft->get_nu());
  // Getting the state dimension from calc() call
  modelsoft->calc(datasoft, x, f, u);
  modelfree->calc(datafree, x, u);
  // BOOST_CHECK((data->xout - datafree->xout).norm() <= 1e-8);
  // std::cout << "size soft xout = " << data->xout.size() << std::endl;
  // std::cout << "size free xout = " << datafree->xout.size() << std::endl;
  // std::cout << data->xout - datafree->xout << std::endl;
  BOOST_CHECK((datasoft->xout - datafree->xout).isZero(1e-4));
}

void test_calc_equivalent_free(DAMSoftContact3DTypes::Type action_type,
                               PinocchioReferenceTypes::Type ref_type) {
  // create the model
  DAMSoftContact3DFactory factory;
  boost::shared_ptr<sobec::DAMSoftContact3DAugmentedFwdDynamics> modelsoft =
      factory.create(action_type, ref_type);
  
  // Set 0 stiffness and damping
  modelsoft->set_Kp(0.);
  modelsoft->set_Kv(0.);
  // Set active contact
  modelsoft->set_active_contact(true);
  // Test that freeFwdDyn = SoftContact(Kp=0;Kv=0), without armature
  test_calc_free(modelsoft, Eigen::VectorXd::Zero(modelsoft->get_state()->get_nv()));
  // Test that freeFwdDyn = SoftContact(Kp=0;Kv=0), WITH armature
  test_calc_free(modelsoft, 1e-3*Eigen::VectorXd::Ones(modelsoft->get_state()->get_nv()));

  boost::shared_ptr<sobec::DAMSoftContact3DAugmentedFwdDynamics> modelsoft2 =
      factory.create(action_type, ref_type);
  // Set non-zero stiffness and damping
  modelsoft2->set_Kp(1000.);
  modelsoft2->set_Kv(70.);
  // Set inactive contact
  modelsoft2->set_active_contact(false);
  // Test that freeFwdDyn = SoftContact(active=False) without armature
  test_calc_free(modelsoft2, Eigen::VectorXd::Zero(modelsoft2->get_state()->get_nv()));
  // Test that freeFwdDyn = SoftContact(active=False) WITH armature
  test_calc_free(modelsoft2, 1e-3*Eigen::VectorXd::Ones(modelsoft2->get_state()->get_nv()));
}


void test_calcDiff_free(boost::shared_ptr<sobec::DAMSoftContact3DAugmentedFwdDynamics> modelsoft, 
                        Eigen::VectorXd armature){
  // Create DAM free
  boost::shared_ptr<crocoddyl::StateMultibody> statemb = boost::static_pointer_cast<crocoddyl::StateMultibody>(modelsoft->get_state()); 
  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> modelfree =
      boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(
          statemb, modelsoft->get_actuation(), modelsoft->get_costs());
  // Add gravity cost on free model
  boost::shared_ptr<crocoddyl::CostModelAbstract> cost = boost::make_shared<crocoddyl::CostModelResidual>(
          statemb, boost::make_shared<crocoddyl::ResidualModelControlGrav>( statemb, modelfree->get_actuation()->get_nu() ));
  modelfree->get_costs()->addCost( "grav_reg", cost, 0.01);
  // optional armature
  if(!armature.isZero(1e-9)){
    if(modelsoft->get_state()->get_nv() > modelsoft->get_nu()){
      armature.head(6) = Eigen::VectorXd::Zero(6);
    }
    modelfree->set_armature(armature);
    modelsoft->set_armature(armature);
  }
  boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract> datafree = modelfree->createData();
  boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract> datasoft = modelsoft->createData();
  // Generating random state and control vectors
  const Eigen::VectorXd x = modelsoft->get_state()->rand();
  const Eigen::VectorXd f = Eigen::Vector3d::Zero();
  const Eigen::VectorXd u = Eigen::VectorXd::Random(modelsoft->get_nu());
  // Set 0 stiffness and damping
  modelsoft->set_Kp(0.);
  modelsoft->set_Kv(0.);
  // Getting the state dimension from calc() call
  modelsoft->calc(datasoft, x, f, u);
  modelsoft->calcDiff(datasoft, x, f, u);
  modelfree->calc(datafree, x, u);
  modelfree->calcDiff(datafree, x, u);
  // Checking the partial derivatives against NumDiff
  double tol = 1e-6;
  BOOST_CHECK((datasoft->Fx - datafree->Fx).isZero(tol));
  BOOST_CHECK((datasoft->Fu - datafree->Fu).isZero(tol));
  BOOST_CHECK((datasoft->Lx - datafree->Lx).isZero(tol));
  BOOST_CHECK((datasoft->Lu - datafree->Lu).isZero(tol));
  BOOST_CHECK((datasoft->Lxx - datafree->Lxx).isZero(tol));
  BOOST_CHECK((datasoft->Lxu - datafree->Lxu).isZero(tol));
  BOOST_CHECK((datasoft->Luu - datafree->Luu).isZero(tol));
}

void test_calcDiff_equivalent_free(DAMSoftContact3DTypes::Type action_type,
                               PinocchioReferenceTypes::Type ref_type) {
  // create the model
  DAMSoftContact3DFactory factory;
  boost::shared_ptr<sobec::DAMSoftContact3DAugmentedFwdDynamics> modelsoft =
      factory.create(action_type, ref_type);
  // test_calcDiff_free(modelsoft, Eigen::VectorXd::Zero(modelsoft->get_state()->get_nv()));
  // Set 0 stiffness and damping
  modelsoft->set_Kp(0.);
  modelsoft->set_Kv(0.);
  // Set active contact
  modelsoft->set_active_contact(true);
  // Test that freeFwdDyn = SoftContact(Kp=0;Kv=0), without armature
  test_calcDiff_free(modelsoft, Eigen::VectorXd::Zero(modelsoft->get_state()->get_nv()));
  // Test that freeFwdDyn = SoftContact(Kp=0;Kv=0), WITH armature
  test_calcDiff_free(modelsoft, 1e-3*Eigen::VectorXd::Ones(modelsoft->get_state()->get_nv()));

  boost::shared_ptr<sobec::DAMSoftContact3DAugmentedFwdDynamics> modelsoft2 =
      factory.create(action_type, ref_type);
  // Set non-zero stiffness and damping
  modelsoft2->set_Kp(1000.);
  modelsoft2->set_Kv(70.);
  // Set inactive contact
  modelsoft2->set_active_contact(false);
  // Test that freeFwdDyn = SoftContact(active=False) without armature
  test_calcDiff_free(modelsoft2, Eigen::VectorXd::Zero(modelsoft2->get_state()->get_nv()));
  // Test that freeFwdDyn = SoftContact(active=False) WITH armature
  test_calcDiff_free(modelsoft2, 1e-3*Eigen::VectorXd::Ones(modelsoft2->get_state()->get_nv()));

}

//----------------------------------------------------------------------------//

void register_action_model_unit_tests(DAMSoftContact3DTypes::Type action_type,
                                      PinocchioReferenceTypes::Type ref_type = PinocchioReferenceTypes::LOCAL) {
  boost::test_tools::output_test_stream test_name;
  test_name << "test_" << action_type << "_" << ref_type;
  std::cout << "Running " << test_name.str() << std::endl;
  test_suite* ts = BOOST_TEST_SUITE(test_name.str());
  ts->add(BOOST_TEST_CASE(boost::bind(&test_check_data, action_type, ref_type)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_calc_returns_state, action_type, ref_type)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_calc_returns_a_cost, action_type, ref_type)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_partial_derivatives_against_numdiff, action_type, ref_type)));
  // Test equivalence with Euler for soft contact when Kp, Kv = 0 and f=0, or active_contact = 0
  ts->add(BOOST_TEST_CASE(boost::bind(&test_calc_equivalent_free, action_type, ref_type)));
  ts->add(BOOST_TEST_CASE(boost::bind(&test_calcDiff_equivalent_free, action_type, ref_type)));
  // test partials with armature 
  ts->add(BOOST_TEST_CASE(boost::bind(&test_partial_derivatives_against_numdiff_armature, action_type, ref_type)));
  framework::master_test_suite().add(ts);
}

bool init_function() {

  for (size_t i = 0; i < DAMSoftContact3DTypes::all.size(); ++i) {
    for (size_t j = 0; j < PinocchioReferenceTypes::all.size(); ++j) {
    register_action_model_unit_tests(DAMSoftContact3DTypes::all[i], PinocchioReferenceTypes::all[j]);
    }
  }

  return true;
}

int main(int argc, char** argv) {
  return ::boost::unit_test::unit_test_main(&init_function, argc, argv);
}
