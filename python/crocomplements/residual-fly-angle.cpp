///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2022, LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "sobec/crocomplements/residual-fly-angle.hpp"

#include <boost/python.hpp>
#include <boost/python/enum.hpp>
#include <eigenpy/eigenpy.hpp>
#include <pinocchio/multibody/fwd.hpp>  // Must be included first!

#include "sobec/fwd.hpp"

namespace sobec {
namespace python {

using namespace crocoddyl;
namespace bp = boost::python;

void exposeResidualFlyAngle() {
  bp::register_ptr_to_python<boost::shared_ptr<ResidualModelFlyAngle> >();

  bp::class_<ResidualModelFlyAngle, bp::bases<ResidualModelAbstract> >(
      "ResidualModelFlyAngle",
      bp::init<boost::shared_ptr<StateMultibody>, pinocchio::FrameIndex, double, double, double, double, 
               std::size_t>(
          bp::args("self", "state", "frame_id", "slope", "height", "dist", "width", "nu"),
          "Initialize the residual model.\n\n"
          ":param state: state of the multibody system\n"
          ":param frame_id: reference colliding frame\n"
          ":param slope: slope ie altitude multiplier."
          ":param height: height of sigmoid."
          ":param dist: translation of sigmoid."
          ":param width: contraction parameter."
          ":param nu: dimension of control vector"))
      .def<void (ResidualModelFlyAngle::*)(
          const boost::shared_ptr<ResidualDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &ResidualModelFlyAngle::calc,
          bp::args("self", "data", "x", "u"),
          "Compute the residual.\n\n"
          ":param data: residual data\n"
          ":param x: time-discrete state vector\n"
          ":param u: time-discrete control input")
      .def<void (ResidualModelFlyAngle::*)(
          const boost::shared_ptr<ResidualDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &ResidualModelAbstract::calc, bp::args("self", "data", "x"))
      .def<void (ResidualModelFlyAngle::*)(
          const boost::shared_ptr<ResidualDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &ResidualModelFlyAngle::calcDiff,
          bp::args("self", "data", "x", "u"),
          "Compute the Jacobians of the residual.\n\n"
          "It assumes that calc has been run first.\n"
          ":param data: action data\n"
          ":param x: time-discrete state vector\n"
          ":param u: time-discrete control input\n")
      .def<void (ResidualModelFlyAngle::*)(
          const boost::shared_ptr<ResidualDataAbstract>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &ResidualModelAbstract::calcDiff,
          bp::args("self", "data", "x"))
      .def("createData", &ResidualModelFlyAngle::createData,
           bp::with_custodian_and_ward_postcall<0, 2>(),
           bp::args("self", "data"),
           "Create the residual data.\n\n"
           "Each residual model has its own data that needs to be allocated. "
           "This function\n"
           "returns the allocated data for a predefined residual.\n"
           ":param data: shared data\n"
           ":return residual data.")
      .add_property("slope", &ResidualModelFlyAngle::getSlope,
                    &ResidualModelFlyAngle::setSlope,
                    "Set slope (ie altitude multiplicator)");

  bp::register_ptr_to_python<boost::shared_ptr<ResidualModelFlyAngle> >();

  bp::class_<ResidualDataFlyAngle, bp::bases<ResidualDataAbstract> >(
      "ResidualDataFlyAngle", "Data for vel collision residual.\n\n",
      bp::init<ResidualModelFlyAngle*, DataCollectorAbstract*>(
          bp::args("self", "model", "data"),
          "Create vel collision residual data.\n\n"
          ":param model: pair collision residual model\n"
          ":param data: shared data")[bp::with_custodian_and_ward<
          1, 2, bp::with_custodian_and_ward<1, 3> >()])
      .add_property("pinocchio",
                    bp::make_getter(&ResidualDataFlyAngle::pinocchio,
                                    bp::return_internal_reference<>()),
                    "pinocchio data");
}

}  // namespace python
}  // namespace sobec
