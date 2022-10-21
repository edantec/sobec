#include "sobec/hand_table/model_factory_hand.hpp"

#include <crocoddyl/multibody/fwd.hpp>

#include "sobec/crocomplements/residual-cop.hpp"
#include "sobec/walk-with-traj/designer.hpp"

namespace sobec {

ModelMakerHand::ModelMakerHand() {}

ModelMakerHand::ModelMakerHand(const ModelMakerHandSettings &settings,
                               const RobotDesigner &designer) {
  initialize(settings, designer);
}

void ModelMakerHand::initialize(const ModelMakerHandSettings &settings,
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

void ModelMakerHand::defineFeetContact(Contact &contactCollector) {
  boost::shared_ptr<crocoddyl::ContactModelAbstract> ContactModelLeft =
      boost::make_shared<crocoddyl::ContactModel6D>(
          state_, designer_.get_LF_id(), designer_.get_LF_frame(),
          actuation_->get_nu(), eVector2(0., 50.));

  boost::shared_ptr<crocoddyl::ContactModelAbstract> ContactModelRight =
      boost::make_shared<crocoddyl::ContactModel6D>(
          state_, designer_.get_RF_id(), designer_.get_RF_frame(),
          actuation_->get_nu(), eVector2(0., 50.));

  contactCollector->addContact(designer_.get_LF_name(), ContactModelLeft,
                               true);
  contactCollector->addContact(designer_.get_RF_name(), ContactModelRight,
                               true);
}

void ModelMakerHand::defineCoMTask(Cost &costCollector) {
  boost::shared_ptr<crocoddyl::CostModelAbstract> comCost =
      boost::make_shared<crocoddyl::CostModelResidual>(
          state_, boost::make_shared<crocoddyl::ResidualModelCoMPosition>(
              state_, designer_.get_com_position(), actuation_->get_nu()));
  costCollector.get()->addCost("comTask", comCost,
                               settings_.wCoM, true);
}

void ModelMakerHand::defineHandContact(Contact &contactCollector,
                                   const Phase &phase) {
  boost::shared_ptr<crocoddyl::ContactModelAbstract> ContactModelHandRight =
      boost::make_shared<crocoddyl::ContactModel3D>(
          state_, designer_.get_RH_id(), designer_.get_RH_frame().translation(),
          actuation_->get_nu(), eVector2(0., 4.));
          
  boost::shared_ptr<crocoddyl::ContactModelAbstract> ContactModelHandLeft =
      boost::make_shared<crocoddyl::ContactModel3D>(
          state_, designer_.get_LH_id(), designer_.get_LH_frame().translation(),
          actuation_->get_nu(), eVector2(0., 4.));
          
  contactCollector->addContact(designer_.get_RH_name(), ContactModelHandRight,
                               false);
  contactCollector->addContact(designer_.get_LH_name(), ContactModelHandLeft,
                               false);
                               
  if (phase == Phase::CONTACT_RIGHT)
    contactCollector->changeContactStatus(designer_.get_RH_name(), true);
  if (phase == Phase::CONTACT_LEFT)
    contactCollector->changeContactStatus(designer_.get_LH_name(), true);
}

void ModelMakerHand::defineFeetWrenchCost(Cost &costCollector) {
  double Mg = -designer_.getRobotMass() * settings_.gravity(2);
  double Fz_ref = Mg / 2;

  crocoddyl::WrenchCone wrenchCone_LF =
      crocoddyl::WrenchCone(Eigen::Matrix3d::Identity(), settings_.mu, settings_.coneBox,
                            4, true, settings_.minNforce, settings_.maxNforce);
  crocoddyl::WrenchCone wrenchCone_RF =
      crocoddyl::WrenchCone(Eigen::Matrix3d::Identity(), settings_.mu, settings_.coneBox,
                            4, true, settings_.minNforce, settings_.maxNforce);

  eVector6 refWrench_LF = eVector6::Zero();
  eVector6 refWrench_RF = eVector6::Zero();
  refWrench_LF(2) = Fz_ref;
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
                               settings_.wWrenchCone, true);
  costCollector.get()->addCost("wrench_RF", wrenchModel_RF,
                               settings_.wWrenchCone, true);
}

void ModelMakerHand::defineHandForceTask(Cost &costCollector, const Phase &phase) {
  Eigen::VectorXd pin_force(6);
  pin_force << 0, 0, 50, 0, 0, 0;
  pinocchio::Force refForce = pinocchio::Force(pin_force);
  boost::shared_ptr<crocoddyl::ResidualModelContactForce> forceResidualModelRight = 
      boost::make_shared<crocoddyl::ResidualModelContactForce>(state_,
            designer_.get_RH_id(),designer_.get_RH_frame().actInv(refForce),3,actuation_->get_nu());
  boost::shared_ptr<crocoddyl::ResidualModelContactForce> forceResidualModelLeft = 
      boost::make_shared<crocoddyl::ResidualModelContactForce>(state_,
            designer_.get_LH_id(),designer_.get_LH_frame().actInv(refForce),3,actuation_->get_nu());
  
  
  boost::shared_ptr<crocoddyl::CostModelAbstract> forceModel_RH = 
            boost::make_shared<crocoddyl::CostModelResidual>(state_, forceResidualModelRight);
  boost::shared_ptr<crocoddyl::CostModelAbstract> forceModel_LH = 
            boost::make_shared<crocoddyl::CostModelResidual>(state_, forceResidualModelLeft);
  
  costCollector.get()->addCost("force_LH", forceModel_LH,
                               settings_.wForceHand, false);
  costCollector.get()->addCost("force_RH", forceModel_RH,
                               settings_.wForceHand, false);
  if (phase == Phase::CONTACT_LEFT)
    costCollector.get()->changeCostStatus("force_LH",true);
  if (phase == Phase::CONTACT_RIGHT)
    costCollector.get()->changeCostStatus("force_RH",true);
}

void ModelMakerHand::definePostureTask(Cost &costCollector) {
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

void ModelMakerHand::defineActuationTask(Cost &costCollector) {
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

void ModelMakerHand::defineJointLimits(Cost &costCollector) {
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

void ModelMakerHand::defineDCMTask(Cost &costCollector, const Phase &phase) {
	
  Eigen::Vector3d ref_position = Eigen::VectorXd::Zero(3);
  ref_position = (designer_.get_RF_frame().translation() + 
	              designer_.get_LF_frame().translation()) / 2.;
  
  if (phase == Phase::CONTACT_LEFT) {
	  ref_position = (ref_position + designer_.get_LH_frame().translation()) / 2;
  }
  if (phase == Phase::CONTACT_RIGHT) {
	  ref_position = (ref_position + designer_.get_RH_frame().translation()) / 2;
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

void ModelMakerHand::defineHandTracking(Cost &costCollector, const Phase &phase) {
  boost::shared_ptr<crocoddyl::ActivationModelQuadFlatLog> activationQF =
      boost::make_shared<crocoddyl::ActivationModelQuadFlatLog>(3, 0.1);

  boost::shared_ptr<crocoddyl::ResidualModelFrameTranslation>
      residualRH =
          boost::make_shared<crocoddyl::ResidualModelFrameTranslation>(
              state_, designer_.get_RH_id(), designer_.get_RH_frame().translation(),
              actuation_->get_nu());
  boost::shared_ptr<crocoddyl::ResidualModelFrameTranslation>
      residualLH =
          boost::make_shared<crocoddyl::ResidualModelFrameTranslation>(
              state_, designer_.get_LH_id(), designer_.get_LH_frame().translation(),
              actuation_->get_nu());
  
  boost::shared_ptr<crocoddyl::CostModelAbstract> trackingModelRH =
      boost::make_shared<crocoddyl::CostModelResidual>(state_, activationQF,
                                                       residualRH);
  boost::shared_ptr<crocoddyl::CostModelAbstract> trackingModelLH =
      boost::make_shared<crocoddyl::CostModelResidual>(state_, activationQF,
                                                       residualLH);
                                                       
  costCollector.get()->addCost("placement_RH", trackingModelRH,
                               settings_.wHandPlacement, false);
  costCollector.get()->addCost("placement_LH", trackingModelLH,
                               settings_.wHandPlacement, false);
  if (phase == Phase::CONTACT_RIGHT || phase == Phase::TRACKING_RIGHT)
    costCollector.get()->changeCostStatus("placement_RH", true);
  if (phase == Phase::CONTACT_LEFT || phase == Phase::TRACKING_LEFT)
    costCollector.get()->changeCostStatus("placement_LH", true);
}

AMA ModelMakerHand::formulateHandTracker(const Phase &phase) {
  Contact contacts = boost::make_shared<crocoddyl::ContactModelMultiple>(
      state_, actuation_->get_nu());
  Cost costs =
      boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());

  defineFeetContact(contacts);
  defineHandContact(contacts, phase);

  defineJointLimits(costs);
  definePostureTask(costs);
  defineActuationTask(costs);
  defineHandTracking(costs, phase);
  defineHandForceTask(costs,phase);
  defineFeetWrenchCost(costs); 
  defineCoMTask(costs);

  DAM runningDAM =
      boost::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>(
          state_, actuation_, contacts, costs, 0., true);
  AMA runningModel = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(
      runningDAM, settings_.timeStep);

  return runningModel;
}

AMA ModelMakerHand::formulateTerminalHandTracker(const Phase &phase) {
  Contact contacts = boost::make_shared<crocoddyl::ContactModelMultiple>(
      state_, actuation_->get_nu());
  Cost costs =
      boost::make_shared<crocoddyl::CostModelSum>(state_, actuation_->get_nu());

  defineFeetContact(contacts);

  defineJointLimits(costs);
  definePostureTask(costs);
  defineDCMTask(costs, phase);

  DAM terminalDAM =
      boost::make_shared<crocoddyl::DifferentialActionModelContactFwdDynamics>(
          state_, actuation_, contacts, costs, 0., true);
  AMA terminalModel = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(
      terminalDAM, 0);

  return terminalModel;
}

std::vector<AMA> ModelMakerHand::formulateHorizon(
    const std::vector<Phase> &phases) {
  // for loop to generate a vector of IAMs
  std::vector<AMA> models;
  for (std::size_t i = 0; i < phases.size(); i++) {
	models.push_back(formulateHandTracker(phases[i]));
  }

  return models;
}

std::vector<AMA> ModelMakerHand::formulateHorizon(const int &T) {
  std::vector<Phase> phases(T, NO_HAND);
  return formulateHorizon(phases);
}

}  // namespace sobec
