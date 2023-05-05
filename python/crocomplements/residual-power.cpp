///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2021, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "sobec/crocomplements/residual-power.hpp"

#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <eigenpy/eigenpy.hpp>
#include <pinocchio/multibody/fwd.hpp>  // Must be included first!

#include "sobec/fwd.hpp"

namespace sobec {
namespace python {

using namespace crocoddyl;
// using namespace crocoddyl::python;
namespace bp = boost::python;

void exposeResidualPower() {
  bp::register_ptr_to_python<boost::shared_ptr<ResidualModelPower> >();

  bp::class_<ResidualModelPower, bp::bases<ResidualModelAbstract> >(
      "ResidualModelPower",
      "This residual function defines the power consumption "
      "r = u * v ",
      bp::init<boost::shared_ptr<StateMultibody>, std::size_t>(
          bp::args("self", "state", "nu"),
          "Initialize the power residual model.\n\n"
          ":param state: state of the multibody system\n"
          ":param nu: dimension of control vector"))
      .def<void (ResidualModelPower::*)(const boost::shared_ptr<ResidualDataAbstract>&,
                                        const Eigen::Ref<const Eigen::VectorXd>&,
                                        const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &ResidualModelPower::calc, bp::args("self", "data", "x", "u"),
          "Compute the power residual.\n\n"
          ":param data: residual data\n"
          ":param x: time-discrete state vector\n"
          ":param u: time-discrete control input")
      .def<void (ResidualModelPower::*)(const boost::shared_ptr<ResidualDataAbstract>&,
                                        const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &ResidualModelAbstract::calc, bp::args("self", "data", "x"))
      .def<void (ResidualModelPower::*)(const boost::shared_ptr<ResidualDataAbstract>&,
                                        const Eigen::Ref<const Eigen::VectorXd>&,
                                        const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &ResidualModelPower::calcDiff, bp::args("self", "data", "x", "u"),
          "Compute the Jacobians of the power residual.\n\n"
          "It assumes that calc has been run first.\n"
          ":param data: action data\n"
          ":param x: time-discrete state vector\n"
          ":param u: time-discrete control input\n")
      .def<void (ResidualModelPower::*)(const boost::shared_ptr<ResidualDataAbstract>&,
                                        const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &ResidualModelAbstract::calcDiff, bp::args("self", "data", "x"))
      .def("createData", &ResidualModelPower::createData, bp::with_custodian_and_ward_postcall<0, 2>(),
           bp::args("self", "data"),
           "Create the power residual data.\n\n"
           "Each residual model has its own data that needs to be allocated. "
           "This function\n"
           "returns the allocated data for a predefined residual.\n"
           ":param data: shared data\n"
           ":return residual data.");

  bp::register_ptr_to_python<boost::shared_ptr<ResidualDataPower> >();

  bp::class_<ResidualDataPower, bp::bases<ResidualDataAbstract> >(
      "ResidualDataPower", "Data for power residual.\n\n",
      bp::init<ResidualModelPower*, DataCollectorAbstract*>(
          bp::args("self", "model", "data"),
          "Create power residual data.\n\n"
          ":param model: power residual model")[bp::with_custodian_and_ward<1, 2, bp::with_custodian_and_ward<1, 3> >()]);
}

}  // namespace python
}  // namespace sobec
