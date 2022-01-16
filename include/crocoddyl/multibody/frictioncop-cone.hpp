///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2020-2021, University of Edinburgh, University of Oxford
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef CROCODDYL_MULTIBODY_FRICTIONCOP_CONE_HPP_
#define CROCODDYL_MULTIBODY_FRICTIONCOP_CONE_HPP_

#include "crocoddyl/multibody/fwd.hpp"
#include "crocoddyl/core/mathbase.hpp"
#include "crocoddyl/core/utils/deprecate.hpp"

namespace crocoddyl {

/**
 * @brief This class encapsulates a friction-cop cone
 *
 * A friction-cop cone is a 5D polyhedral convex cone that characterizes feasible contact wrench.
 * The friction-cop cone is derived in the case of rectangular support areas, which is of practical importance since most
 * humanoid robot feet can be adequately approximated by rectangles. For more details read:
 *
 * /sa `FrictionConeTpl`, `CoPSupportTpl`
 */
template <typename _Scalar>
class FrictionCopConeTpl {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef typename MathBase::Vector2s Vector2s;
  typedef typename MathBase::Vector3s Vector3s;
  typedef typename MathBase::Matrix3s Matrix3s;
  typedef typename MathBase::Matrix6s Matrix6s;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::MatrixXs MatrixXs;
  typedef typename MathBase::MatrixX3s MatrixX3s;
  typedef typename MathBase::MatrixX6s MatrixX6s;
  typedef typename MathBase::Quaternions Quaternions;

  /**
   * @brief Initialize the friction-cop cone
   */
  explicit FrictionCopConeTpl();

  /**
   * @brief Initialize the friction-cop cone
   *
   * @param[in] R           Rotation matrix that defines the cone orientation w.r.t. the inertial frame
   * @param[in] mu          Friction coefficient
   * @param[in] box         Dimension of the foot surface dim = (length, width)
   * @param[in] nf          Number of facets (default 4)
   * @param[in] inner_appr  Label that describes the type of friction cone approximation (inner/outer)
   * @param[in] min_nforce  Minimum normal force (default 0.)
   * @param[in] max_nforce  Maximum normal force (default inf number))
   */
  FrictionCopConeTpl(const Matrix3s& R, const Scalar mu, const Vector2s& box, const std::size_t nf = 4,
                const bool inner_appr = true, const Scalar min_nforce = Scalar(0.),
                const Scalar max_nforce = std::numeric_limits<Scalar>::infinity());
  DEPRECATED("Use constructor that includes inner_appr",
             FrictionCopConeTpl(const Matrix3s& R, const Scalar mu, const Vector2s& box, std::size_t nf,
                           const Scalar min_nforce,
                           const Scalar max_nforce = std::numeric_limits<Scalar>::infinity());)

  /**
   * @brief Initialize the friction-cop cone
   *
   * @param[in] cone  Friction-cop cone
   */
  FrictionCopConeTpl(const FrictionCopConeTpl<Scalar>& cone);
  ~FrictionCopConeTpl();

  /**
   * @brief Update the matrix of friction-cop cone inequalities in the world frame.
   *
   * This matrix-vector pair describes the linearized Coulomb friction model as follow:
   * \f$ -ub \leq A \times w \leq -lb \f$,
   * where friction-cop, \f$ w \f$, is expressed in the inertial frame located at the
   * center of the rectangular foot contact area (length, width) with axes parallel to
   * those of the world frame.
   */
  void update();
  DEPRECATED("Use update().", void update(const Matrix3s& R, const Scalar mu, const Vector2s& box, 
                                          const Scalar min_nforce = Scalar(0.),
                                          const Scalar max_nforce = std::numeric_limits<Scalar>::infinity()));

  /**
   * @brief Return the matrix of friction-cop cone
   */
  const MatrixX6s& get_A() const;

  /**
   * @brief Return the lower bound of inequalities
   */
  const VectorXs& get_lb() const;

  /**
   * @brief Return the upper bound of inequalities
   */
  const VectorXs& get_ub() const;

  /**
   * @brief Return the number of facets
   */
  std::size_t get_nf() const;

  /**
   * @brief Return the rotation matrix that defines the cone orientation w.r.t. the inertial frame
   */
  const Matrix3s& get_R() const;

  /**
   * @brief Return dimension of the foot surface dim = (length, width)
   */
  const Vector2s& get_box() const;

  /**
   * @brief Return friction coefficient
   */
  const Scalar get_mu() const;

  /**
   * @brief Return the label that describes the type of friction cone approximation (inner/outer)
   */
  bool get_inner_appr() const;

  /**
   * @brief Return the minimum normal force
   */
  const Scalar get_min_nforce() const;

  /**
   * @brief Return the maximum normal force
   */
  const Scalar get_max_nforce() const;

  /**
   * @brief Modify the rotation matrix that defines the cone orientation w.r.t. the inertial frame
   *
   * Note that you need to run `update` for updating the inequality matrix and bounds.
   */
  void set_R(const Matrix3s& R);

  /**
   * @brief Modify dimension of the foot surface dim = (length, width)
   *
   * Note that you need to run `update` for updating the inequality matrix and bounds.
   */
  void set_box(const Vector2s& box);

  /**
   * @brief Modify friction coefficient
   *
   * Note that you need to run `update` for updating the inequality matrix and bounds.
   */
  void set_mu(const Scalar mu);

  /**
   * @brief Modify the label that describes the type of friction cone approximation (inner/outer)
   *
   * Note that you need to run `update` for updating the inequality matrix and bounds.
   */
  void set_inner_appr(const bool inner_appr);

  /**
   * @brief Modify the minium normal force
   *
   * Note that you need to run `update` for updating the inequality matrix and bounds.
   */
  void set_min_nforce(const Scalar min_nforce);

  /**
   * @brief Modify the maximum normal force
   *
   * Note that you need to run `update` for updating the inequality matrix and bounds.
   */
  void set_max_nforce(const Scalar max_nforce);

  FrictionCopConeTpl<Scalar>& operator=(const FrictionCopConeTpl<Scalar>& other);

  template <class Scalar>
  friend std::ostream& operator<<(std::ostream& os, const FrictionCopConeTpl<Scalar>& X);

 private:
  std::size_t nf_;     //!< Number of facets
  MatrixX6s A_;        //!< Matrix of friction-cop cone
  VectorXs ub_;        //!< Upper bound of the friction-cop cone
  VectorXs lb_;        //!< Lower bound of the friction-cop cone
  Matrix3s R_;         //!< Rotation of the friction-cop cone w.r.t. the inertial frame
  Vector2s box_;       //!< Dimension of the foot surface (length, width)
  Scalar mu_;          //!< Friction coefficient
  bool inner_appr_;    //!< Label that describes the type of friction cone approximation (inner/outer)
  Scalar min_nforce_;  //!< Minimum normal force
  Scalar max_nforce_;  //!< Maximum normal force
};

}  // namespace crocoddyl

#include "crocoddyl/multibody/frictioncop-cone.hxx"

#endif  // CROCODDYL_MULTIBODY_FRICTIONCOP_CONE_HPP_
