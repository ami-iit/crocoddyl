///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2020-2021, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "crocoddyl/multibody/residuals/contact-frictioncop-cone.hpp"
#include "python/crocoddyl/multibody/multibody.hpp"

namespace crocoddyl {
namespace python {

void exposeResidualContactFrictionCopCone() {
  bp::register_ptr_to_python<boost::shared_ptr<ResidualModelContactFrictionCopCone> >();

  bp::class_<ResidualModelContactFrictionCopCone, bp::bases<ResidualModelAbstract> >(
      "ResidualModelContactFrictionCopCone",
      bp::init<boost::shared_ptr<StateMultibody>, pinocchio::FrameIndex, FrictionCopCone, std::size_t>(
          bp::args("self", "state", "id", "fref", "nu"),
          "Initialize the contact friction-cop cone residual model.\n\n"
          ":param state: state of the multibody system\n"
          ":param id: reference frame id\n"
          ":param fref: contact friction-cop cone\n"
          ":param nu: dimension of control vector"))
      .def(bp::init<boost::shared_ptr<StateMultibody>, pinocchio::FrameIndex, FrictionCopCone>(
          bp::args("self", "state", "id", "fref"),
          "Initialize the contact friction-cop cone residual model.\n\n"
          "For this case the default nu is equals to model.nv.\n"
          ":param state: state of the multibody system\n"
          ":param id: reference frame id\n"
          ":param fref: contact friction-cop cone"))
      .def<void (ResidualModelContactFrictionCopCone::*)(const boost::shared_ptr<ResidualDataAbstract>&,
                                                    const Eigen::Ref<const Eigen::VectorXd>&,
                                                    const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &ResidualModelContactFrictionCopCone::calc, bp::args("self", "data", "x", "u"),
          "Compute the contact friction-cop cone residual.\n\n"
          ":param data: residual data\n"
          ":param x: time-discrete state vector\n"
          ":param u: time-discrete control input")
      .def<void (ResidualModelContactFrictionCopCone::*)(const boost::shared_ptr<ResidualDataAbstract>&,
                                                    const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &ResidualModelAbstract::calc, bp::args("self", "data", "x"))
      .def<void (ResidualModelContactFrictionCopCone::*)(const boost::shared_ptr<ResidualDataAbstract>&,
                                                    const Eigen::Ref<const Eigen::VectorXd>&,
                                                    const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &ResidualModelContactFrictionCopCone::calcDiff, bp::args("self", "data", "x", "u"),
          "Compute the Jacobians of the contact friction-cop cone residual.\n\n"
          "It assumes that calc has been run first.\n"
          ":param data: action data\n"
          ":param x: time-discrete state vector\n"
          ":param u: time-discrete control input\n")
      .def<void (ResidualModelContactFrictionCopCone::*)(const boost::shared_ptr<ResidualDataAbstract>&,
                                                    const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &ResidualModelAbstract::calcDiff, bp::args("self", "data", "x"))
      .def("createData", &ResidualModelContactFrictionCopCone::createData, bp::with_custodian_and_ward_postcall<0, 2>(),
           bp::args("self", "data"),
           "Create the contact friction-cop cone residual data.\n\n"
           "Each residual model has its own data that needs to be allocated. This function\n"
           "returns the allocated data for the contact friction-cop cone residual.\n"
           ":param data: shared data\n"
           ":return residual data.")
      .add_property("id", &ResidualModelContactFrictionCopCone::get_id, &ResidualModelContactFrictionCopCone::set_id,
                    "reference frame id")
      .add_property(
          "reference",
          bp::make_function(&ResidualModelContactFrictionCopCone::get_reference, bp::return_internal_reference<>()),
          &ResidualModelContactFrictionCopCone::set_reference, "reference contact friction-cop cone");

  bp::register_ptr_to_python<boost::shared_ptr<ResidualDataContactFrictionCopCone> >();

  bp::class_<ResidualDataContactFrictionCopCone, bp::bases<ResidualDataAbstract> >(
      "ResidualDataContactFrictionCopCone", "Data for contact friction-cop cone residual.\n\n",
      bp::init<ResidualModelContactFrictionCopCone*, DataCollectorAbstract*>(
          bp::args("self", "model", "data"),
          "Create contact friction-cop cone residual data.\n\n"
          ":param model: contact friction-cop cone residual model\n"
          ":param data: shared data")[bp::with_custodian_and_ward<1, 2, bp::with_custodian_and_ward<1, 3> >()])
      .add_property(
          "contact",
          bp::make_getter(&ResidualDataContactFrictionCopCone::contact, bp::return_value_policy<bp::return_by_value>()),
          bp::make_setter(&ResidualDataContactFrictionCopCone::contact),
          "contact data associated with the current residual");
}

}  // namespace python
}  // namespace crocoddyl
