///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2020-2021, University of Edinburgh, INRIA
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef CROCODDYL_MULTIBODY_COSTS_CONTACT_FRICTIONCOP_CONE_HPP_
#define CROCODDYL_MULTIBODY_COSTS_CONTACT_FRICTIONCOP_CONE_HPP_

#include "crocoddyl/multibody/fwd.hpp"
#include "crocoddyl/core/costs/residual.hpp"
#include "crocoddyl/multibody/states/multibody.hpp"
#include "crocoddyl/multibody/residuals/contact-frictioncop-cone.hpp"
#include "crocoddyl/core/utils/exception.hpp"

#include "crocoddyl/multibody/frames-deprecated.hpp"

namespace crocoddyl {

#pragma GCC diagnostic push  // TODO: Remove once the deprecated FrameXX has been removed in a future release
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

template <typename _Scalar>
class CostModelContactFrictionCopConeTpl : public CostModelResidualTpl<_Scalar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef CostModelResidualTpl<Scalar> Base;
  typedef StateMultibodyTpl<Scalar> StateMultibody;
  typedef ActivationModelAbstractTpl<Scalar> ActivationModelAbstract;
  typedef ResidualModelContactFrictionCopConeTpl<Scalar> ResidualModelContactFrictionCopCone;
  typedef FrameFrictionCopConeTpl<Scalar> FrameFrictionCopCone;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::MatrixXs MatrixXs;
  typedef typename MathBase::MatrixX6s MatrixX6s;
  typedef typename MathBase::MatrixX5s MatrixX5s;

  CostModelContactFrictionCopConeTpl(boost::shared_ptr<StateMultibody> state,
                                boost::shared_ptr<ActivationModelAbstract> activation, const FrameFrictionCopCone& fref,
                                const std::size_t nu);
  CostModelContactFrictionCopConeTpl(boost::shared_ptr<StateMultibody> state,
                                boost::shared_ptr<ActivationModelAbstract> activation, const FrameFrictionCopCone& fref);
  CostModelContactFrictionCopConeTpl(boost::shared_ptr<StateMultibody> state, const FrameFrictionCopCone& fref,
                                const std::size_t nu);
  CostModelContactFrictionCopConeTpl(boost::shared_ptr<StateMultibody> state, const FrameFrictionCopCone& fref);
  virtual ~CostModelContactFrictionCopConeTpl();

 protected:
  virtual void set_referenceImpl(const std::type_info& ti, const void* pv);
  virtual void get_referenceImpl(const std::type_info& ti, void* pv);

  using Base::activation_;
  using Base::nu_;
  using Base::residual_;
  using Base::state_;
  using Base::unone_;

 private:
  FrameFrictionCopCone fref_;
};

}  // namespace crocoddyl

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "crocoddyl/multibody/costs/contact-frictioncop-cone.hxx"

#pragma GCC diagnostic pop

#endif  // CROCODDYL_MULTIBODY_COSTS_CONTACT_FRICTIONCOP_CONE_HPP_
