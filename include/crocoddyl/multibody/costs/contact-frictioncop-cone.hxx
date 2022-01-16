///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2020-2021, University of Edinburgh, INRIA
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "crocoddyl/core/utils/exception.hpp"
#include "crocoddyl/multibody/costs/contact-frictioncop-cone.hpp"

namespace crocoddyl {

template <typename Scalar>
CostModelContactFrictionCopConeTpl<Scalar>::CostModelContactFrictionCopConeTpl(
    boost::shared_ptr<StateMultibody> state, boost::shared_ptr<ActivationModelAbstract> activation,
    const FrameFrictionCopCone& fref, const std::size_t nu)
    : Base(state, activation, boost::make_shared<ResidualModelContactFrictionCopCone>(state, fref.id, fref.cone, nu)),
      fref_(fref) {
  std::cerr << "Deprecated CostModelContactFrictionCopCone: Use ResidualModelContactFrictionCopCone with CostModelResidual"
            << std::endl;
}

template <typename Scalar>
CostModelContactFrictionCopConeTpl<Scalar>::CostModelContactFrictionCopConeTpl(
    boost::shared_ptr<StateMultibody> state, boost::shared_ptr<ActivationModelAbstract> activation,
    const FrameFrictionCopCone& fref)
    : Base(state, activation, boost::make_shared<ResidualModelContactFrictionCopCone>(state, fref.id, fref.cone)),
      fref_(fref) {
  std::cerr << "Deprecated CostModelContactFrictionCopCone:esidualModelContactFrictionCopCone with CostModelResidual"
            << std::endl;
}

template <typename Scalar>
CostModelContactFrictionCopConeTpl<Scalar>::CostModelContactFrictionCopConeTpl(boost::shared_ptr<StateMultibody> state,
                                                                     const FrameFrictionCopCone& fref, const std::size_t nu)
    : Base(state, boost::make_shared<ResidualModelContactFrictionCopCone>(state, fref.id, fref.cone, nu)), fref_(fref) {
  std::cerr << "Deprecated CostModelContactFrictionCopCone: Use ResidualModelContactFrictionCopCone with CostModelResidual"
            << std::endl;
}

template <typename Scalar>
CostModelContactFrictionCopConeTpl<Scalar>::CostModelContactFrictionCopConeTpl(boost::shared_ptr<StateMultibody> state,
                                                                     const FrameFrictionCopCone& fref)
    : Base(state, boost::make_shared<ResidualModelContactFrictionCopCone>(state, fref.id, fref.cone)), fref_(fref) {
  std::cerr << "Deprecated CostModelContactFrictionCopCone: Use ResidualModelContactFrictionCopCone with CostModelResidual"
            << std::endl;
}

template <typename Scalar>
CostModelContactFrictionCopConeTpl<Scalar>::~CostModelContactFrictionCopConeTpl() {}

template <typename Scalar>
void CostModelContactFrictionCopConeTpl<Scalar>::set_referenceImpl(const std::type_info& ti, const void* pv) {
  if (ti == typeid(FrameFrictionCopCone)) {
    fref_ = *static_cast<const FrameFrictionCopCone*>(pv);
    ResidualModelContactFrictionCopCone* residual = static_cast<ResidualModelContactFrictionCopCone*>(residual_.get());
    residual->set_id(fref_.id);
    residual->set_reference(fref_.cone);
  } else {
    throw_pretty("Invalid argument: incorrect type (it should be FrameFrictionCopCone)");
  }
}

template <typename Scalar>
void CostModelContactFrictionCopConeTpl<Scalar>::get_referenceImpl(const std::type_info& ti, void* pv) {
  if (ti == typeid(FrameFrictionCopCone)) {
    FrameFrictionCopCone& ref_map = *static_cast<FrameFrictionCopCone*>(pv);
    ResidualModelContactFrictionCopCone* residual = static_cast<ResidualModelContactFrictionCopCone*>(residual_.get());
    fref_.id = residual->get_id();
    fref_.cone = residual->get_reference();
    ref_map = fref_;
  } else {
    throw_pretty("Invalid argument: incorrect type (it should be FrameFrictionCopCone)");
  }
}

}  // namespace crocoddyl
