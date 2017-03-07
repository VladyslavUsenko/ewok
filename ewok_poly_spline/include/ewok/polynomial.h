/**
* This file is part of Ewok.
*
* Copyright 2017 Vladyslav Usenko, Technical University of Munich.
* Developed by Vladyslav Usenko <vlad dot usenko at tum dot de>,
* for more information see <http://vision.in.tum.de/research/robotvision/replanning>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Ewok is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Ewok is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Ewok. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef EWOK_POLY_SPLINE_INCLUDE_EWOK_POLYNOMIAL_H_
#define EWOK_POLY_SPLINE_INCLUDE_EWOK_POLYNOMIAL_H_

#include <Eigen/Dense>

#include <memory>
#include <array>

namespace ewok {

// Class implements a polynomial with degree _N-1.
//
// Implementation is based on [1]:
// "Polynomial Trajectory Planning for Quadrotor Flight"
// C. Richter, A. Bry and N. Roy
// http://rss2012_workshop.visual-navigation.com/pdf/richter_rss13_workshop.pdf

template<int _N, typename _Scalar = double>
class Polynomial {
 public:
  static const int N = _N;
  static const int DEG = N - 1;

  typedef std::shared_ptr <Polynomial<_N, _Scalar>> Ptr;

  typedef Eigen::Matrix<_Scalar, 1, _N> VectorNT;
  typedef Eigen::Matrix<_Scalar, _N, 1> VectorN;
  typedef Eigen::Matrix <_Scalar, _N, _N> MatrixN;
  typedef std::array<MatrixN, 4> MatrixNArray;

  Polynomial() {
    static_assert(_N % 2 == 0, "Polynomial degree must be even number");
  }

  template<class Derived>
  explicit Polynomial(const Eigen::MatrixBase <Derived> &coeffs) {
    static_assert(_N % 2 == 0, "Polynomial degree must be even number");
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, N);
    coefficients_ = coeffs;
  }


  template<class Derived>
  void getCoeffs(const Eigen::MatrixBase <Derived> &coeffs_const) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, N);

    Eigen::MatrixBase <Derived> &coeffs =
        const_cast<Eigen::MatrixBase <Derived> &>(coeffs_const);

    coeffs = coefficients_;
  }

  template<class Derived>
  void setCoeffs(const Eigen::MatrixBase <Derived> &coeffs_const) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, N);

    Eigen::MatrixBase <Derived> &coeffs =
        const_cast<Eigen::MatrixBase <Derived> &>(coeffs_const);

    coefficients_ = coeffs;
  }

  inline _Scalar evaluate(_Scalar t, int derivative) const {
    // return evaluate_standard(t, derivative);
    return evaluate_horner(t, derivative);
  }

  template<class Derived>
  static void quadraticCostJacobian(const Eigen::MatrixBase <Derived> &Q_const,
                                    _Scalar t, int derivative) {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, N, N);
    Eigen::MatrixBase <Derived> &Q =
        const_cast<Eigen::MatrixBase <Derived> &>(Q_const);

    Q.setZero();

    _Scalar _tc = t;

    for (int i = derivative; i < N; i++) {
      _Scalar _tr = _tc;

      for (int j = derivative; j < N; j++) {
        Q(i, j) = quadratic_coefficients_[derivative - 1](i, j) * _tr;
        _tr *= t;
      }
      _tc *= t;
    }
  }

  static MatrixN quadraticCostJacobian(_Scalar t, int derivative) {
    MatrixN Q;
    quadraticCostJacobian(Q, t, derivative);
    return Q;
  }

  template<class Derived>
  static void baseCoeffsWithTime(const Eigen::MatrixBase <Derived> &res_const,
                                 _Scalar t, int derivative) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, N);
    Eigen::MatrixBase <Derived> &res =
        const_cast<Eigen::MatrixBase <Derived> &>(res_const);

    res.setZero();
    res[derivative] = base_coefficients_(derivative, derivative);

    if (t == 0) return;

    _Scalar _t = t;
    for (int j = derivative + 1; j < N; j++) {
      res[j] = base_coefficients_(derivative, j) * _t;
      _t = _t * t;
    }
  }

  static VectorNT baseCoeffsWithTime(_Scalar t, int derivative) {
    VectorNT res;
    baseCoeffsWithTime(res, t, derivative);
    return res;
  }

  template<class Derived>
  static void endpointConstrainsMatrix(
      const Eigen::MatrixBase <Derived> &A_const, _Scalar t) {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, N, N);
    Eigen::MatrixBase <Derived> &A =
        const_cast<Eigen::MatrixBase <Derived> &>(A_const);

    A.setZero();

    for (int i = 0; i < N / 2; i++) {
      A(i, i) = base_coefficients_(i, i);
    }

    for (int i = 0; i < N / 2; i++) {
      baseCoeffsWithTime(A.row(N / 2 + i), t, i);
    }
  }

  static MatrixN endpointConstrainsMatrix(_Scalar t) {
    MatrixN A;
    endpointConstrainsMatrix(A, t);
    return A;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;


  // Base coefficients to evaluate the value and derivatives of the
  // polynomial.
  static const MatrixN base_coefficients_;

  // Base coefficients for integral over
  // squared derivative of velocity, acceleration, jerk, snap
  // Eq. (14) in [1].
  static const MatrixNArray quadratic_coefficients_;

 protected:
  inline _Scalar evaluate_standard(_Scalar t, int derivative) const {
    _Scalar tp = 1;
    _Scalar res = 0;

    for (int i = derivative; i < N; ++i) {
      res += coefficients_[i] * base_coefficients_(derivative, i) * tp;
      tp *= t;
    }

    return res;
  }

  inline _Scalar evaluate_horner(_Scalar t, int derivative) const {
    _Scalar res = 0;

    for (int i = N-1; i >= derivative; --i) {
      res = res * t + coefficients_[i] * base_coefficients_(derivative, i);
    }

    return res;
  }

  VectorNT coefficients_;
};

template<int _N, typename _Scalar>
typename Polynomial<_N, _Scalar>::MatrixN computeBaseCoefficients() {
  typename Polynomial<_N, double>::MatrixN base_coefficients;

  base_coefficients.setZero();
  base_coefficients.row(0).setOnes();

  const int DEG = Polynomial<_N, _Scalar>::DEG;
  int order = DEG;
  for (int n = 1; n < _N; n++) {
    for (int i = DEG - order; i < _N; i++) {
      base_coefficients(n, i) = (order - DEG + i) * base_coefficients(n - 1, i);
    }
    order--;
  }
  return base_coefficients.template cast<_Scalar>();
}

template<int _N, typename _Scalar>
typename Polynomial<_N, _Scalar>::MatrixNArray computeQuadraticCoefficients() {
  typename Polynomial<_N, _Scalar>::MatrixNArray res;
  typename Polynomial<_N, _Scalar>::MatrixN base_coefficients =
      computeBaseCoefficients<_N, _Scalar>();

  const int DEG = Polynomial<_N, _Scalar>::DEG;

  for (int derivative = 1; derivative < 5; derivative++) {
    res[derivative - 1].setZero();

    for (int col = 0; col < _N - derivative; col++) {
      for (int row = 0; row < _N - derivative; row++) {
        _Scalar exp = (DEG - derivative) * 2 + 1 - row - col;

        res[derivative - 1](DEG - row, DEG - col) =
            base_coefficients(derivative, _N - 1 - row) *
            base_coefficients(derivative, _N - 1 - col) * 2 / exp;
      }
    }
  }

  return res;
}

template<int _N, typename _Scalar>
const typename Polynomial<_N, _Scalar>::MatrixN
    Polynomial<_N, _Scalar>::base_coefficients_ =
    computeBaseCoefficients<_N, _Scalar>();

template<int _N, typename _Scalar>
const typename Polynomial<_N, _Scalar>::MatrixNArray
    Polynomial<_N, _Scalar>::quadratic_coefficients_ =
    computeQuadraticCoefficients<_N, _Scalar>();

}  // namespace ewok

#endif  // EWOK_POLY_SPLINE_INCLUDE_EWOK_POLYNOMIAL_H_
