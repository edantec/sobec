#include <pinocchio/fwd.hpp>  // Must be included first!
// keep this line on top
#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <boost/python/return_internal_reference.hpp>
#include <crocoddyl/core/activation-base.hpp>
#include <eigenpy/eigenpy.hpp>
#include <sobec/walk-with-traj/model_factory.hpp>

#include "sobec/fwd.hpp"

namespace sobec {
namespace python {
namespace bp = boost::python;

template <typename T>
inline void py_list_to_std_vector(const bp::object &iterable,
                                  std::vector<T> &out) {
  out = std::vector<T>(boost::python::stl_input_iterator<T>(iterable),
                       boost::python::stl_input_iterator<T>());
}

template <class T>
bp::list std_vector_to_py_list(const std::vector<T> &v) {
  bp::object get_iter = bp::iterator<std::vector<T> >();
  bp::object iter = get_iter(v);
  bp::list l(iter);
  return l;
}

void initialize(ModelMaker &self, const bp::dict &settings,
                const RobotDesigner &designer) {
  ModelMakerSettings conf;

  // timing
  conf.timeStep = bp::extract<double>(settings["timeStep"]);

  // physics
  conf.gravity = bp::extract<eVector3>(settings["gravity"]);
  conf.mu = bp::extract<double>(settings["mu"]);
  conf.coneBox = bp::extract<eVector2>(settings["coneBox"]);
  conf.minNforce = bp::extract<double>(settings["minNforce"]);
  conf.maxNforce = bp::extract<double>(settings["maxNforce"]);
  conf.comHeight = bp::extract<double>(settings["comHeight"]);
  conf.omega = bp::extract<double>(settings["omega"]);
  conf.footSize = bp::extract<double>(settings["footSize"]);

  // gains
  conf.wFootPlacement = bp::extract<double>(settings["wFootPlacement"]);
  conf.wHandPlacement = bp::extract<double>(settings["wHandPlacement"]);
  conf.wStateReg = bp::extract<double>(settings["wStateReg"]);
  conf.wControlReg = bp::extract<double>(settings["wControlReg"]);
  conf.wLimit = bp::extract<double>(settings["wLimit"]);
  conf.wWrenchCone = bp::extract<double>(settings["wWrenchCone"]);
  conf.wForceTask = bp::extract<double>(settings["wForceTask"]);
  conf.wCoP = bp::extract<double>(settings["wCoP"]);
  conf.wDCM = bp::extract<double>(settings["wDCM"]);
  conf.stateWeights = bp::extract<Eigen::VectorXd>(settings["stateWeights"]);
  conf.controlWeights = bp::extract<Eigen::VectorXd>(settings["controlWeights"]);
  conf.forceWeights = bp::extract<Eigen::VectorXd>(settings["forceWeights"]);
  conf.lowKinematicLimits = bp::extract<Eigen::VectorXd>(settings["lowKinematicLimits"]);
  conf.highKinematicLimits = bp::extract<Eigen::VectorXd>(settings["highKinematicLimits"]);
  conf.th_grad = bp::extract<double>(settings["th_grad"]);
  conf.th_stop = bp::extract<double>(settings["th_stop"]);

  self.initialize(conf, designer);
}

bp::dict get_settings(ModelMaker &self) {
  ModelMakerSettings conf = self.get_settings();
  bp::dict settings;
  settings["timeStep"] = conf.timeStep;
  settings["gravity"] = conf.gravity;
  settings["mu"] = conf.mu;
  settings["coneBox"] = conf.coneBox;
  settings["minNforce"] = conf.minNforce;
  settings["maxNforce"] = conf.maxNforce;
  settings["comHeight"] = conf.comHeight;
  settings["omega"] = conf.omega;
  settings["footSize"] = conf.footSize;
  settings["wFootPlacement"] = conf.wFootPlacement;
  settings["wHandPlacement"] = conf.wHandPlacement;
  settings["wStateReg"] = conf.wStateReg;
  settings["wControlReg"] = conf.wControlReg;
  settings["wLimit"] = conf.wLimit;
  settings["wWrenchCone"] = conf.wWrenchCone;
  settings["wForceTask"] = conf.wForceTask;
  settings["wCoP"] = conf.wCoP;
  settings["wDCM"] = conf.wDCM;
  settings["stateWeights"] = conf.stateWeights;
  settings["controlWeights"] = conf.controlWeights;
  settings["forceWeights"] = conf.forceWeights;
  settings["lowKinematicLimits"] = conf.lowKinematicLimits;
  settings["highKinematicLimits"] = conf.highKinematicLimits;
  settings["th_grad"] = conf.th_grad;
  settings["th_stop"] = conf.th_stop;
  return settings;
}

bp::list formulateHorizon(ModelMaker &self,
                          const bp::list &supports = bp::list(),
                          const int &length = 0) {
  if (bp::len(supports) > 0) {
    std::vector<Support> contacts;
    py_list_to_std_vector(supports, contacts);
    std::vector<AMA> models = self.formulateHorizon(contacts);

    return std_vector_to_py_list(models);

  } else if (length > 0) {
    std::vector<AMA> models = self.formulateHorizon(length);
    return std_vector_to_py_list(models);
  } else {
    throw std::runtime_error(
        "Either a list of supports or an horizon length must be provided.");
  }
}

bp::list formulateHandHorizon(ModelMaker &self,
                          const bp::list &supports = bp::list()) {
  std::vector<Support> contacts;
  py_list_to_std_vector(supports, contacts);
  std::vector<AMA> models = self.formulateHandHorizon(contacts);

  return std_vector_to_py_list(models);
}

void defineFeetContact(ModelMaker &self,
                       crocoddyl::ContactModelMultiple &contactCollector,
                       const Support &supports = Support::DOUBLE) {
  Contact contacts =
      boost::make_shared<crocoddyl::ContactModelMultiple>(contactCollector);
  self.defineFeetContact(contacts, supports);
  contactCollector = *contacts;
}

void defineFeetWrenchCost(ModelMaker &self,
                          crocoddyl::CostModelSum &costCollector,
                          const Support &supports = Support::DOUBLE) {
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(costCollector);
  self.defineFeetWrenchCost(costs, supports);
  costCollector = *costs;
}

void defineFeetForceTask(ModelMaker &self,
                          crocoddyl::CostModelSum &costCollector,
                          const Support &supports = Support::DOUBLE) {
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(costCollector);
  self.defineFeetForceTask(costs, supports);
  costCollector = *costs;
}

void defineFeetTracking(ModelMaker &self,
                        crocoddyl::CostModelSum &costCollector,
                        const Support &supports = Support::DOUBLE) {
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(costCollector);
  self.defineFeetTracking(costs, supports);
  costCollector = *costs;
}

void defineHandTracking(ModelMaker &self,
                        crocoddyl::CostModelSum &costCollector) {
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(costCollector);
  self.defineHandTracking(costs);
  costCollector = *costs;
}

void defineDCMTask(ModelMaker &self,
                        crocoddyl::CostModelSum &costCollector) {
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(costCollector);
  self.defineDCMTask(costs);
  costCollector = *costs;
}

void definePostureTask(ModelMaker &self,
                       crocoddyl::CostModelSum &costCollector) {
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(costCollector);
  self.definePostureTask(costs);
  costCollector = *costs;
}

void defineActuationTask(ModelMaker &self,
                         crocoddyl::CostModelSum &costCollector) {
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(costCollector);
  self.defineActuationTask(costs);
  costCollector = *costs;
}

void defineJointLimits(ModelMaker &self,
                       crocoddyl::CostModelSum &costCollector) {
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(costCollector);
  self.defineJointLimits(costs);
  costCollector = *costs;
}

void defineCoPTask(ModelMaker &self,
                   crocoddyl::CostModelSum &costCollector,
                   const Support &supports = Support::DOUBLE) {
  Cost costs = boost::make_shared<crocoddyl::CostModelSum>(costCollector);
  self.defineCoPTask(costs, supports);
  costCollector = *costs;
}

void exposeModelFactory() {
  bp::enum_<Support>("Support")
      .value("LEFT", Support::LEFT)
      .value("LEFT", Support::RIGHT)
      .value("DOUBLE", Support::DOUBLE)
      .value("HAND", Support::HAND);

  bp::class_<ModelMaker>("ModelMaker", bp::init<>())
      .def("initialize", &initialize, bp::args("self", "settings", "design"))
      .def("get_settings", &get_settings, bp::args("self"))
      .def("defineFeetContact", &defineFeetContact,
           (bp::arg("self"), bp::arg("contactCollector"),
            bp::arg("supports") = Support::DOUBLE))
      .def("defineFeetWrenchCost", &defineFeetWrenchCost,
           (bp::arg("self"), bp::arg("costCollector"),
            bp::arg("supports") = Support::DOUBLE))
      .def("defineFeetForceTask", &defineFeetForceTask,
           (bp::arg("self"), bp::arg("costCollector"),
            bp::arg("supports") = Support::DOUBLE))
      .def("defineFeetTracking", &defineFeetTracking,
           (bp::arg("self"), bp::arg("costCollector"),
            bp::arg("supports") = Support::DOUBLE))
      .def("defineHandTracking", &defineHandTracking,
           bp::arg("self"), bp::arg("costCollector"))
      .def("defineDCMTask", &defineDCMTask,
           bp::arg("self"), bp::arg("costCollector"))
      .def("definePostureTask", &definePostureTask,
           bp::args("self", "costCollector"))
      .def("defineActuationTask", &defineActuationTask,
           bp::args("self", "costCollector"))
      .def("defineJointLimits", &defineJointLimits,
           bp::args("self", "costCollector"))
      .def("defineCoPTask", &defineCoPTask,
           (bp::arg("self"), bp::arg("costCollector"),
            bp::arg("supports") = Support::DOUBLE))
      .def("formulateStepTracker", &ModelMaker::formulateStepTracker,
           (bp::arg("self"), bp::arg("supports") = Support::DOUBLE))
      .def("formulateHandTracker", &ModelMaker::formulateHandTracker,
           (bp::arg("self"), bp::arg("supports") = Support::DOUBLE))
      .def("formulateTerminalStepTracker", &ModelMaker::formulateTerminalStepTracker,
           (bp::arg("self"), bp::arg("supports") = Support::DOUBLE))
      .def("formulateTerminalHandTracker", &ModelMaker::formulateTerminalHandTracker,
           (bp::arg("self"), bp::arg("supports") = Support::DOUBLE))
      .def("getState", &ModelMaker::getState, bp::args("self"))
      .def("setState", &ModelMaker::setState, bp::args("self"))
      .def("getActuation", &ModelMaker::getActuation, bp::args("self"))
      .def("setActuation", &ModelMaker::setActuation, bp::args("self"))
      .def("formulateHorizon", &formulateHorizon,
           (bp::arg("self"), bp::arg("supports") = bp::list(),
            bp::arg("length") = 0))
      .def("formulateHandHorizon", &formulateHandHorizon,
           (bp::arg("self"), bp::arg("supports") = bp::list()));
}
}  // namespace python
}  // namespace sobec
