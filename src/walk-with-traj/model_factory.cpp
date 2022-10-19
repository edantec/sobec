#include "sobec/walk-with-traj/model_factory.hpp"

#include <crocoddyl/multibody/fwd.hpp>

#include "sobec/crocomplements/residual-cop.hpp"
#include "sobec/walk-with-traj/designer.hpp"

namespace sobec {

ModelMaker::ModelMaker() {}

ModelMaker::ModelMaker(const ModelMakerSettings &settings,
                       const RobotDesigner &designer) {
  initialize(settings, designer);
}

void ModelMaker::initialize(const ModelMakerSettings &settings,
                            const RobotDesigner &designer) {
  settings_ = settings;
  designer_ = designer;

  state_ = boost::make_shared<crocoddyl::StateMultibody>(
      boost::make_shared<pinocchio::Model>(designer_.get_rModel()));
  actuation_ =
      boost::make_shared<crocoddyl::ActuationModelFloatingBase>(state_);

  x0_.resize(designer_.get_rModel().nq + designer_.get_rModel().nv);
  x0_ << designer_.get_q0(), Eigen::VectorXd::Zero(designer_.get_rModel().nv);
  initialized_ = true;
}

void ModelMaker::defineFeetContact(Contact &contactCollector,
                                   const Support &support) {
  boost::shared_ptr<crocoddyl::ContactModelAbstract> ContactModelLeft =
      boost::make_shared<crocoddyl::ContactModel6D>(
          state_, designer_.get_LF_id(), designer_.get_LF_frame(),
          actuation_->get_nu(), eVector2(0., 50.));

  boost::shared_ptr<crocoddyl::ContactModelAbstract> ContactModelRight =
      boost::make_shared<crocoddyl::ContactModel6D>(
          state_, designer_.get_RF_id(), designer_.get_RF_frame(),
          actuation_->get_nu(), eVector2(0., 50.));

  contactCollector->addContact(designer_.get_LF_name(), ContactModelLeft,
                               false);
  contactCollector->addContact(designer_.get_RF_name(), ContactModelRight,
                               false);

  if (support == Support::LEFT || support == Support::DOUBLE || support == Support::HAND)
    contactCollector->changeContactStatus(designer_.get_LF_name(), true);

  if (support == Support::RIGHT || support == Support::DOUBLE || support == Support::HAND)
    contactCollector->changeContactStatus(designer_.get_RF_name(), true);
}

void ModelMaker::defineHandContact(Contact &contactCollector,
                                   const Support &support) {
  boost::shared_ptr<crocoddyl::ContactModelAbstract> ContactModelHand =
      boost::make_shared<crocoddyl::ContactModel3D>(
          state_, designer_.get_RH_id(), designer_.get_RH_frame().translation(),
          actuation_->get_nu(), eVector2(0., 50.));

  contactCollector->addContact(designer_.get_RH_name(), ContactModelHand,
                               false);

  if (support == Support::HAND)
    contactCollector->changeContactStatus(designer_.get_RH_name(), true);
}

void ModelMaker::defineFeetWrenchCost(Cost &costCollector,
                                      const Support &support) {
  double Mg = -designer_.getRobotMass() * settings_.gravity(2);
  double Fz_ref;
  support == Support::DOUBLE ? Fz_ref = Mg / 2 : Fz_ref = Mg;

  /*Eigen::Matrix3d coneRotationLeft =
      designer_.get_LF_frame().rotation().transpose();
  Eigen::Matrix3d coneRotationRight =
      designer_.get_RF_frame().rotation().transpose();*/

  crocoddyl::WrenchCone wrenchCone_LF =
      crocoddyl::WrenchCone(Eigen::Matrix3d::Identity(), settings_.mu, settings_.coneBox,
                            4, true, settings_.minNforce, settings_.maxNforce);
  crocoddyl::WrenchCone wrenchCone_RF =
      crocoddyl::WrenchCone(Eigen::Matrix3d::Identity(), settings_.mu, settings_.coneBox,
                            4, true, settings_.minNforce, settings_.maxNforce);

  eVector6 refWrench_LF = eVector6::Zero();
  eVector6 refWrench_RF = eVector6::Zero();
  if (support == Support::LEFT || support == Support::DOUBLE || support == Support::HAND)
    refWrench_LF(2) = Fz_ref;
  if (support == Support::RIGHT || support == Support::DOUBLE || support == Support::HAND)
    refWrench_RF(2) = Fz_ref;

  Eigen::VectorXd refCost_LF = wrenchCone_LF.get_A() * refWrench_LF;
  Eigen::VectorXd refCost_RF = wrenchCone_LF.get_A() * refWrench_RF;

  boost::shared_ptr<ActivationModelQuadRef> activation_LF_Wrench =
      boost::make_shared<ActivationModelQuadRef>(refCost_LF);
  boost::shared_ptr<ActivationModelQuadRef> activation_RF_Wrench =
      boost::make_shared<ActivationModelQuadRef>(refCost_RF);

  boost::shared_ptr<crocoddyl::ResidualModelContactWrenchCone>
      residual_LF_Wrench =
          boost::make_shared<crocoddyl::ResidualModelContactWrenchCone>(
              state_, designer_.get_LF_id(), wrenchCone_LF,
              actuation_->get_nu());
  boost::shared_ptr<crocoddyl::CostModelAbstract> wrenchModel_LF =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_, activation_LF_Wrench, residual_LF_Wrench);

  boost::shared_ptr<crocoddyl::ResidualModelContactWrenchCone>
      residual_RF_Wrench =
          boost::make_shared<crocoddyl::ResidualModelContactWrenchCone>(
              state_, designer_.get_RF_id(), wrenchCone_RF,
              actuation_->get_nu());
  boost::shared_ptr<crocoddyl::CostModelAbstract> wrenchModel_RF =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_, activation_RF_Wrench, residual_RF_Wrench);

  costCollector.get()->addCost("wrench_LF", wrenchModel_LF,
                               settings_.wWrenchCone, false);
  costCollector.get()->addCost("wrench_RF", wrenchModel_RF,
                               settings_.wWrenchCone, false);
  if (support == Support::LEFT || support == Support::DOUBLE || support == Support::HAND)
    costCollector.get()->changeCostStatus("wrench_LF",true);
  if (support == Support::RIGHT || support == Support::DOUBLE || support == Support::HAND)
    costCollector.get()->changeCostStatus("wrench_RF",true);
}

void ModelMaker::defineFeetTracking(Cost &costCollector, const Support &support) {
  boost::shared_ptr<crocoddyl::ActivationModelQuadFlatLog> activationQF =
      boost::make_shared<crocoddyl::ActivationModelQuadFlatLog>(6, 0.01);

  boost::shared_ptr<crocoddyl::ResidualModelFramePlacement>
      residual_LF_Tracking =
          boost::make_shared<crocoddyl::ResidualModelFramePlacement>(
              state_, designer_.get_LF_id(), designer_.get_LF_frame(),
              actuation_->get_nu());

  boost::shared_ptr<crocoddyl::ResidualModelFramePlacement>
      residual_RF_Tracking =
          boost::make_shared<crocoddyl::ResidualModelFramePlacement>(
              state_, designer_.get_RF_id(), designer_.get_RF_frame(),
              actuation_->get_nu());

  boost::shared_ptr<crocoddyl::CostModelAbstract> trackingModel_LF =
      boost::make_shared<crocoddyl::CostModelResidual>(state_, activationQF,
                                                       residual_LF_Tracking);
  boost::shared_ptr<crocoddyl::CostModelAbstract> trackingModel_RF =
      boost::make_shared<crocoddyl::CostModelResidual>(state_, activationQF,
                                                       residual_RF_Tracking);
  
  costCollector.get()->addCost("placement_LF", trackingModel_LF,
                               settings_.wFootPlacement, false);
  costCollector.get()->addCost("placement_RF", trackingModel_RF,
                               settings_.wFootPlacement, false);
  if (support == Support::LEFT || support == Support::DOUBLE)
    costCollector.get()->changeCostStatus("placement_RF",true);
  if (support == Support::RIGHT || support == Support::DOUBLE)
    costCollector.get()->changeCostStatus("placement_LF",true);
}

void ModelMaker::defineFeetForceTask(Cost &costCollector, const Support &support) {
  double Mg = -designer_.getRobotMass() * settings_.gravity(2);
  double Fz_ref;
  support == Support::DOUBLE ? Fz_ref = Mg / 2 : Fz_ref = Mg;
  
  boost::shared_ptr<crocoddyl::ActivationModelWeightedQuad> activationForce =
      boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(
          settings_.forceWeights);

  eVector6 refWrench_LF = eVector6::Zero();
  eVector6 refWrench_RF = eVector6::Zero();
  if (support == Support::LEFT || support == Support::DOUBLE)
    refWrench_LF(2) = Fz_ref;
  if (support == Support::RIGHT || support == Support::DOUBLE)
    refWrench_RF(2) = Fz_ref;
  
  pinocchio::Force refForce_LF = pinocchio::Force(refWrench_LF);
  pinocchio::Force refForce_RF = pinocchio::Force(refWrench_RF);
  
  boost::shared_ptr<crocoddyl::ResidualModelContactForce>
      residual_LF_Force =
          boost::make_shared<crocoddyl::ResidualModelContactForce>(
              state_, designer_.get_LF_id(), refForce_LF, 6,
              actuation_->get_nu());

  boost::shared_ptr<crocoddyl::ResidualModelContactForce>
      residual_RF_Force =
          boost::make_shared<crocoddyl::ResidualModelContactForce>(
              state_, designer_.get_RF_id(), refForce_RF, 6,
              actuation_->get_nu());

  boost::shared_ptr<crocoddyl::CostModelAbstract> forceModel_LF =
      boost::make_shared<crocoddyl::CostModelResidual>(state_, activationForce,
                                                       residual_LF_Force);
  boost::shared_ptr<crocoddyl::CostModelAbstract> forceModel_RF =
      boost::make_shared<crocoddyl::CostModelResidual>(state_, activationForce,
                                                       residual_RF_Force);
  
  costCollector.get()->addCost("force_LF", forceModel_LF,
                               settings_.wForceTask, false);
  costCollector.get()->addCost("force_RF", forceModel_RF,
                               settings_.wForceTask, false);
  if (support == Support::RIGHT || support == Support::DOUBLE || support == Support::HAND)
    costCollector.get()->changeCostStatus("force_RF",true);
  if (support == Support::LEFT || support == Support::DOUBLE || support == Support::HAND)
    costCollector.get()->changeCostStatus("force_LF",true);
}

void ModelMaker::definePostureTask(Cost &costCollector) {
  if (settings_.stateWeights.size() != designer_.get_rModel().nv * 2) {
    throw std::invalid_argument("State weight size is wrong ");
  }
  boost::shared_ptr<crocoddyl::ActivationModelWeightedQuad> activationWQ =
      boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(
          settings_.stateWeights);

  boost::shared_ptr<crocoddyl::CostModelAbstract> postureModel =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_, activationWQ,
          boost::make_shared<crocoddyl::ResidualModelState>(
              state_, x0_, actuation_->get_nu()));

  costCollector.get()->addCost("postureTask", postureModel, settings_.wStateReg,
                               true);
}

void ModelMaker::defineActuationTask(Cost &costCollector) {
  if (settings_.controlWeights.size() != (int)actuation_->get_nu()) {
    throw std::invalid_argument("Control weight size is wrong ");
  }
  boost::shared_ptr<crocoddyl::ActivationModelWeightedQuad> activationWQ =
      boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(
          settings_.controlWeights);  //.tail(actuation->get_nu())

  boost::shared_ptr<crocoddyl::CostModelAbstract> actuationModel =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_, activationWQ,
          boost::make_shared<crocoddyl::ResidualModelControl>(
              state_, actuation_->get_nu()));
  costCollector.get()->addCost("actuationTask", actuationModel,
                               settings_.wControlReg, true);
}

void ModelMaker::defineJointLimits(Cost &costCollector) {
  Eigen::VectorXd lower_bound(2 * state_->get_nv()),
      upper_bound(2 * state_->get_nv());

  double inf = 9999.0;
  lower_bound << Eigen::VectorXd::Constant(6, -inf),
      settings_.lowKinematicLimits,
      Eigen::VectorXd::Constant(state_->get_nv(), -inf); 
  upper_bound << Eigen::VectorXd::Constant(6, inf),
      settings_.highKinematicLimits,
      Eigen::VectorXd::Constant(state_->get_nv(), inf);
  crocoddyl::ActivationBounds bounds =
      crocoddyl::ActivationBounds(lower_bound, upper_bound, 1.);
  
  boost::shared_ptr<crocoddyl::ActivationModelQuadraticBarrier> activationQB =
      boost::make_shared<crocoddyl::ActivationModelQuadraticBarrier>(bounds);
  boost::shared_ptr<crocoddyl::CostModelAbstract> jointLimitCost =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_, activationQB,
          boost::make_shared<crocoddyl::ResidualModelState>(
              state_, actuation_->get_nu()));

  costCollector.get()->addCost("jointLimits", jointLimitCost, settings_.wLimit,
                               true);
}

void ModelMaker::defineDCMTask(Cost &costCollector, const Support &support) {
	
  Eigen::Vector3d ref_position = Eigen::VectorXd::Zero(3);
  if (support == Support::LEFT) {
	  ref_position = designer_.get_LF_frame().translation();
  }
  if (support == Support::RIGHT) {
	  ref_position = designer_.get_RF_frame().translation();
  }
  if (support == Support::DOUBLE || support == Support::HAND) {
	  ref_position = (designer_.get_RF_frame().translation() + 
	                  designer_.get_LF_frame().translation()) / 2.;
  }
  boost::shared_ptr<sobec::ResidualModelDCMPosition>
      residual_DCM =
          boost::make_shared<sobec::ResidualModelDCMPosition>(
              state_, ref_position, 1 / settings_.omega,
              actuation_->get_nu());

  boost::shared_ptr<crocoddyl::CostModelAbstract> DCM_model =
      boost::make_shared<crocoddyl::CostModelResidual>(state_,
                                                       residual_DCM);
  
  costCollector.get()->addCost("DCM", DCM_model,
                               settings_.wDCM, true);
}

void ModelMaker::defineHandTracking(Cost &costCollector) {
  boost::shared_ptr<crocoddyl::ActivationModelQuadFlatLog> activationQF =
      boost::make_shared<crocoddyl::ActivationModelQuadFlatLog>(6, 0.1);

  boost::shared_ptr<crocoddyl::ResidualModelFramePlacement>
      residual_effector_tracking =
          boost::make_shared<crocoddyl::ResidualModelFramePlacement>(
              state_, designer_.get_RH_id(), designer_.get_RH_frame(),
              actuation_->get_nu());

  boost::shared_ptr<crocoddyl::CostModelAbstract> trackingModel =
      boost::make_shared<crocoddyl::CostModelResidual>(state_, activationQF,
                                                       residual_effector_tracking);
  
  costCollector.get()->addCost("placement_RH", trackingModel,
                               settings_.wHandPlacement, true);
}

void ModelMaker::defineCoPTask(Cost &costCollector, const Support &support) {
  Eigen::Vector2d w_cop;
  double value = 1.0 / (settings_.footSize * settings_.footSize);
  w_cop << value, value;

  // LEFT
  boost::shared_ptr<crocoddyl::CostModelAbstract> copCostLF =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_,
          boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(w_cop),
          boost::make_shared<ResidualModelCenterOfPressure>(
              state_, designer_.get_LF_id(), actuation_->get_nu()));

  // RIGHT
  boost::shared_ptr<crocoddyl::CostModelAbstract> copCostRF =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_,
          boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(w_cop),
          boost::make_shared<ResidualModelCenterOfPressure>(
              state_, designer_.get_RF_id(), actuation_->get_nu()));

  costCollector.get()->addCost(designer_.get_LF_name() + "_cop", copCostLF,
                         settings_.wCoP, false);
  costCollector.get()->addCost(designer_.get_RF_name() + "_cop", copCostRF,
                         settings_.wCoP, false);
  if (support == Support::LEFT || support == Support::DOUBLE || support == Support::HAND)
    costCollector.get()->changeCostStatus(designer_.get_LF_name() + "_cop", true);

  if (support == Support::RIGHT || support == Support::DOUBLE || support == Support::HAND)
    costCollector.get()->changeCostStatus(designer_.get_RF_name() + "_cop", true);
}

AMA ModelMaker::formulateStepTracker(const Support &support) {
  Contact contacts = boost::make_shared<crocoddyl::ContactModelMultiple>(
      state_, actuation_->get_nu());
  Cost costs =
      boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());

  defineFeetContact(contacts, support);

  defineJointLimits(costs);
  definePostureTask(costs);
  defineActuationTask(costs);
  defineFeetWrenchCost(costs, support);
  defineFeetForceTask(costs,support);
  defineCoPTask(costs, support);
  defineFeetTracking(costs, support);

  DAM runningDAM =
      boost::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>(
          state_, actuation_, contacts, costs, 0., true);
  AMA runningModel = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(
      runningDAM, settings_.timeStep);

  return runningModel;
}

AMA ModelMaker::formulateHandTracker(const Support &support) {
  Contact contacts = boost::make_shared<crocoddyl::ContactModelMultiple>(
      state_, actuation_->get_nu());
  Cost costs =
      boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());

  defineFeetContact(contacts, support);
  defineHandContact(contacts, support);

  defineJointLimits(costs);
  definePostureTask(costs);
  defineActuationTask(costs);
  defineFeetWrenchCost(costs, support);
  defineCoPTask(costs, support);
  defineHandTracking(costs);

  DAM runningDAM =
      boost::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>(
          state_, actuation_, contacts, costs, 0., true);
  AMA runningModel = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(
      runningDAM, settings_.timeStep);

  return runningModel;
}

AMA ModelMaker::formulateTerminalHandTracker(const Support &support) {
  Contact contacts = boost::make_shared<crocoddyl::ContactModelMultiple>(
      state_, actuation_->get_nu());
  Cost costs =
      boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());

  defineFeetContact(contacts, support);

  defineJointLimits(costs);
  definePostureTask(costs);

  DAM terminalDAM =
      boost::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>(
          state_, actuation_, contacts, costs, 0., true);
  AMA terminalModel = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(
      terminalDAM, 0);

  return terminalModel;
}

AMA ModelMaker::formulateTerminalStepTracker(const Support &support) {
  Contact contacts = boost::make_shared<crocoddyl::ContactModelMultiple>(
      state_, actuation_->get_nu());
  Cost costs =
      boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());

  defineFeetContact(contacts, support);
  defineJointLimits(costs);
  definePostureTask(costs);
  defineFeetTracking(costs, support);
  defineDCMTask(costs, support);

  DAM terminalDAM =
      boost::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>(
          state_, actuation_, contacts, costs, 0., true);
  AMA terminalModel = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(
      terminalDAM, 0);

  return terminalModel;
}

std::vector<AMA> ModelMaker::formulateHorizon(
    const std::vector<Support> &supports) {
  // for loop to generate a vector of IAMs
  std::vector<AMA> models;
  for (std::size_t i = 0; i < supports.size(); i++) {
	models.push_back(formulateStepTracker(supports[i]));
  }

  return models;
}

std::vector<AMA> ModelMaker::formulateHandHorizon(
    const std::vector<Support> &supports) {
  // for loop to generate a vector of IAMs
  std::vector<AMA> models;
  for (std::size_t i = 0; i < supports.size(); i++) {
	models.push_back(formulateHandTracker(supports[i]));
  }

  return models;
}

std::vector<AMA> ModelMaker::formulateHorizon(const int &T) {
  std::vector<Support> supports(T, DOUBLE);
  return formulateHorizon(supports);
}

}  // namespace sobec
