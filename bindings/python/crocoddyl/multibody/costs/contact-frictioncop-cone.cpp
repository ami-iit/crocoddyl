///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2020-2021, University of Edinburgh, INRIA
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "python/crocoddyl/multibody/multibody.hpp"
#include "crocoddyl/multibody/costs/contact-frictioncop-cone.hpp"

namespace crocoddyl {
namespace python {

void exposeCostContactFrictionCopCone() {  // TODO: Remove once the deprecated update call has been removed in a future
                                      // release
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

  bp::register_ptr_to_python<boost::shared_ptr<CostModelContactFrictionCopCone> >();

  bp::class_<CostModelContactFrictionCopCone, bp::bases<CostModelResidual> >(
      "CostModelContactFrictionCopCone",
      bp::init<boost::shared_ptr<StateMultibody>, boost::shared_ptr<ActivationModelAbstract>, FrameFrictionCopCone, int>(
          bp::args("self", "state", "activation", "fref", "nu"),
          "Initialize the contact friction-cop cone cost model.\n\n"
          ":param state: state of the multibody system\n"
          ":param activation: activation model\n"
          ":param fref: frame friction-cop cone\n"
          ":param nu: dimension of control vector"))
      .def(bp::init<boost::shared_ptr<StateMultibody>, boost::shared_ptr<ActivationModelAbstract>, FrameFrictionCopCone>(
          bp::args("self", "state", "activation", "fref"),
          "Initialize the contact friction-cop cone cost model.\n\n"
          "For this case the default nu is equals to model.nv.\n"
          ":param state: state of the multibody system\n"
          ":param activation: activation model\n"
          ":param fref: frame friction-cop cone"))
      .def(bp::init<boost::shared_ptr<StateMultibody>, FrameFrictionCopCone, int>(
          bp::args("self", "state", "fref", "nu"),
          "Initialize the contact friction-cop cone cost model.\n\n"
          "For this case the default activation model is quadratic, i.e.\n"
          "crocoddyl.ActivationModelQuad(6).\n"
          ":param state: state of the multibody system\n"
          ":param fref: frame friction-cop cone\n"
          ":param nu: dimension of control vector"))
      .def(bp::init<boost::shared_ptr<StateMultibody>, FrameFrictionCopCone>(
          bp::args("self", "state", "fref"),
          "Initialize the contact friction-cop cone cost model.\n\n"
          "For this case the default activation model is quadratic, i.e.\n"
          "crocoddyl.ActivationModelQuad(6), and nu is equals to model.nv.\n"
          ":param state: state of the multibody system\n"
          ":param fref: frame friction-cop cone"))
      .add_property("reference", &CostModelContactFrictionCopCone::get_reference<FrameFrictionCopCone>,
                    &CostModelContactFrictionCopCone::set_reference<FrameFrictionCopCone>, "reference frame friction-cop cone");

#pragma GCC diagnostic pop
}

}  // namespace python
}  // namespace crocoddyl
