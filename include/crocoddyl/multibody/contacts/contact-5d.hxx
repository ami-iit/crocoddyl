
///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2021, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "crocoddyl/core/utils/exception.hpp"
#include "crocoddyl/multibody/contacts/contact-5d.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>

namespace crocoddyl {

template <typename Scalar>
ContactModel5DTpl<Scalar>::ContactModel5DTpl(boost::shared_ptr<StateMultibody> state, const pinocchio::FrameIndex id,
                                             const SE3& pref, const std::size_t nu, const Vector2s& gains)
    : Base(state, 5, nu), pref_(pref), gains_(gains) {
  id_ = id;
}

template <typename Scalar>
ContactModel5DTpl<Scalar>::ContactModel5DTpl(boost::shared_ptr<StateMultibody> state, const pinocchio::FrameIndex id,
                                             const SE3& pref, const Vector2s& gains)
    : Base(state, 5), pref_(pref), gains_(gains) {
  id_ = id;
}

#pragma GCC diagnostic push  // TODO: Remove once the deprecated FrameXX has been removed in a future release
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

template <typename Scalar>
ContactModel5DTpl<Scalar>::ContactModel5DTpl(boost::shared_ptr<StateMultibody> state,
                                             const FramePlacementTpl<Scalar>& Mref, const std::size_t nu,
                                             const Vector2s& gains)
    : Base(state, 5, nu), pref_(Mref.placement), gains_(gains) {
  id_ = Mref.id;
  std::cerr << "Deprecated: Use constructor which is not based on FramePlacement." << std::endl;
}

template <typename Scalar>
ContactModel5DTpl<Scalar>::ContactModel5DTpl(boost::shared_ptr<StateMultibody> state,
                                             const FramePlacementTpl<Scalar>& Mref, const Vector2s& gains)
    : Base(state, 5), pref_(Mref.placement), gains_(gains) {
  id_ = Mref.id;
  std::cerr << "Deprecated: Use constructor which is not based on FramePlacement." << std::endl;
}

#pragma GCC diagnostic pop

template <typename Scalar>
ContactModel5DTpl<Scalar>::~ContactModel5DTpl() {}

template <typename Scalar>
void ContactModel5DTpl<Scalar>::calc(const boost::shared_ptr<ContactDataAbstract>& data,
                                     const Eigen::Ref<const VectorXs>&) {
  Data* d = static_cast<Data*>(data.get());
  pinocchio::updateFramePlacement(*state_->get_pinocchio().get(), *d->pinocchio, id_);
  pinocchio::getFrameJacobian(*state_->get_pinocchio().get(), *d->pinocchio, id_, pinocchio::LOCAL, d->fJf);
  d->v = pinocchio::getFrameVelocity(*state_->get_pinocchio().get(), *d->pinocchio, id_);
  d->a = pinocchio::getFrameAcceleration(*state_->get_pinocchio().get(), *d->pinocchio, id_);

  d->Jc = d->fJf.template topRows<5>();
  d->vw = d->v.angular();
  d->vv = d->v.linear();
  //VectorXd a_comp(5);
  //a_comp << -(d->vw[2])*(d->vv[1]), (d->vw[2])*(d->vv[0]), 0, 0, 0;
  //d->a0 = d->(a.toVector()).head(5) + a_comp;
  (d->a0).head(3) = d->a.linear() + d->vw.cross(d->vv);
  (d->a0).tail(2) = (d->a.angular() ).head(2);

  if (gains_[1] != 0.) {
  //VectorXd v_comp(5);
  //v_comp << d->vv[0], d->vv[1], d->vv[2], d->vw[0], d->vw[1];
    d->a0 += gains_[1] * (d->v.toVector() ).head(5); //* v_comp;
  }
}

template <typename Scalar>
void ContactModel5DTpl<Scalar>::calcDiff(const boost::shared_ptr<ContactDataAbstract>& data,
                                         const Eigen::Ref<const VectorXs>&) {
  Data* d = static_cast<Data*>(data.get());
  const pinocchio::JointIndex joint = state_->get_pinocchio()->frames[d->frame].parent;
  pinocchio::getJointAccelerationDerivatives(*state_->get_pinocchio().get(), *d->pinocchio, joint, pinocchio::LOCAL,
                                             d->v_partial_dq, d->a_partial_dq, d->a_partial_dv, d->a_partial_da);
  const std::size_t nv = state_->get_nv();
  pinocchio::skew(d->vv, d->vv_skew);
  pinocchio::skew(d->vw, d->vw_skew);
  d->fXjdv_dq.noalias() = d->fXj * d->v_partial_dq;
  d->fXjda_dq.noalias() = d->fXj * d->a_partial_dq;
  d->fXjda_dv.noalias() = d->fXj * d->a_partial_dv;

  d->da0_dx.topLeftCorner(3,nv).noalias() = d->fXjda_dq.template topRows<3>() +
                                     d->vw_skew * d->fXjdv_dq.template topRows<3>() -
                                     d->vv_skew * d->fXjdv_dq.template bottomRows<3>();
  d->da0_dx.bottomLeftCorner(2,nv).noalias() = d->fXjda_dq.template middleRows<2>(3);

  d->da0_dx.topRightCorner(3,nv).noalias() =
      d->fXjda_dv.template topRows<3>() + d->vw_skew * d->Jc - d->vv_skew * d->fJf.template bottomRows<3>();
  
  d->da0_dx.bottomRightCorner(2,nv).noalias() = d->fXjda_dv.template middleRows<2>(3);
  
  if (gains_[1] != 0.) {
    d->da0_dx.topLeftCorner(3,nv).noalias() += gains_[1] * d->fXj.template topRows<3>() * d->v_partial_dq;
    d->da0_dx.bottomLeftCorner(2,nv).noalias() += gains_[1] * d->fXj.template middleRows<2>(3) * d->v_partial_dq;
    d->da0_dx.topRightCorner(3,nv).noalias() += gains_[1] * d->fXj.template topRows<3>() * d->a_partial_da;
    d->da0_dx.bottomRightCorner(2,nv).noalias() += gains_[1] * d->fXj.template middleRows<2>(3) * d->a_partial_da;
  }
}

template <typename Scalar>
void ContactModel5DTpl<Scalar>::updateForce(const boost::shared_ptr<ContactDataAbstract>& data,
                                            const VectorXs& force) {
  if (force.size() != 5) {
    throw_pretty("Invalid argument: "
                 << "lambda has wrong dimension (it should be 5)");
  }
  Data* d = static_cast<Data*>(data.get());
  data->f = d->jMf.act(pinocchio::ForceTpl<Scalar>(force));
}

template <typename Scalar>
boost::shared_ptr<ContactDataAbstractTpl<Scalar> > ContactModel5DTpl<Scalar>::createData(
    pinocchio::DataTpl<Scalar>* const data) {
  return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this, data);
}

template <typename Scalar>
void ContactModel5DTpl<Scalar>::print(std::ostream& os) const {
  os << "ContactModel5D {frame=" << state_->get_pinocchio()->frames[id_].name << "}";
}

template <typename Scalar>
const pinocchio::SE3Tpl<Scalar>& ContactModel5DTpl<Scalar>::get_reference() const {
  return pref_;
}

#pragma GCC diagnostic push  // TODO: Remove once the deprecated FrameXX has been removed in a future release
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

template <typename Scalar>
FramePlacementTpl<Scalar> ContactModel5DTpl<Scalar>::get_Mref() const {
  return FramePlacementTpl<Scalar>(id_, pref_);
}

#pragma GCC diagnostic pop

template <typename Scalar>
const typename MathBaseTpl<Scalar>::Vector2s& ContactModel5DTpl<Scalar>::get_gains() const {
  return gains_;
}

template <typename Scalar>
void ContactModel5DTpl<Scalar>::set_reference(const SE3& reference) {
  pref_ = reference;
}

}  // namespace crocoddyl
