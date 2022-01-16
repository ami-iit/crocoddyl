///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2020-2021, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "crocoddyl/multibody/residuals/contact-frictioncop-cone.hpp"

namespace crocoddyl {

template <typename Scalar>
ResidualModelContactFrictionCopConeTpl<Scalar>::ResidualModelContactFrictionCopConeTpl(boost::shared_ptr<StateMultibody> state,
                                                                             const pinocchio::FrameIndex id,
                                                                             const FrictionCopCone& fref,
                                                                             const std::size_t nu)
    : Base(state, fref.get_nf() + 5, nu, true, true, true), id_(id), fref_(fref) {}

template <typename Scalar>
ResidualModelContactFrictionCopConeTpl<Scalar>::ResidualModelContactFrictionCopConeTpl(boost::shared_ptr<StateMultibody> state,
                                                                             const pinocchio::FrameIndex id,
                                                                             const FrictionCopCone& fref)
    : Base(state, fref.get_nf() + 5), id_(id), fref_(fref) {}

template <typename Scalar>
ResidualModelContactFrictionCopConeTpl<Scalar>::~ResidualModelContactFrictionCopConeTpl() {}

template <typename Scalar>
void ResidualModelContactFrictionCopConeTpl<Scalar>::calc(const boost::shared_ptr<ResidualDataAbstract>& data,
                                                     const Eigen::Ref<const VectorXs>&,
                                                     const Eigen::Ref<const VectorXs>&) {
  Data* d = static_cast<Data*>(data.get());

  // Compute the residual of the friction-cop cone. Note that we need to transform the friction-cop
  // to the contact frame
  data->r.noalias() = (fref_.get_A()).leftCols(5) * (d->contact->jMf.actInv(d->contact->f).toVector() ).head(5);
}

template <typename Scalar>
void ResidualModelContactFrictionCopConeTpl<Scalar>::calcDiff(const boost::shared_ptr<ResidualDataAbstract>& data,
                                                         const Eigen::Ref<const VectorXs>&,
                                                         const Eigen::Ref<const VectorXs>&) {
  Data* d = static_cast<Data*>(data.get());

  const MatrixXs& df_dx = d->contact->df_dx;
  const MatrixXs& df_du = d->contact->df_du;
  const MatrixX5s& A = (fref_.get_A() ).leftCols(5);
  data->Rx.noalias() = A.leftCols(5) * df_dx.template topRows<5>();
  data->Ru.noalias() = A.leftCols(5) * df_du.template topRows<5>();
}

template <typename Scalar>
boost::shared_ptr<ResidualDataAbstractTpl<Scalar> > ResidualModelContactFrictionCopConeTpl<Scalar>::createData(
    DataCollectorAbstract* const data) {
  return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this, data);
}

template <typename Scalar>
void ResidualModelContactFrictionCopConeTpl<Scalar>::print(std::ostream& os) const {
  boost::shared_ptr<StateMultibody> s = boost::static_pointer_cast<StateMultibody>(state_);
  const Eigen::IOFormat fmt(2, Eigen::DontAlignCols, ", ", ";\n", "", "", "[", "]");
  os << "ResidualModelContactFrictionCopCone {frame=" << s->get_pinocchio()->frames[id_].name << ", mu=" << fref_.get_mu()
     << ", box=" << fref_.get_box().transpose().format(fmt) << "}";
}

template <typename Scalar>
pinocchio::FrameIndex ResidualModelContactFrictionCopConeTpl<Scalar>::get_id() const {
  return id_;
}

template <typename Scalar>
const FrictionCopConeTpl<Scalar>& ResidualModelContactFrictionCopConeTpl<Scalar>::get_reference() const {
  return fref_;
}

template <typename Scalar>
void ResidualModelContactFrictionCopConeTpl<Scalar>::set_id(const pinocchio::FrameIndex id) {
  id_ = id;
}

template <typename Scalar>
void ResidualModelContactFrictionCopConeTpl<Scalar>::set_reference(const FrictionCopCone& reference) {
  fref_ = reference;
}

}  // namespace crocoddyl
