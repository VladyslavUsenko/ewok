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

#ifndef EWOK_POLY_SPLINE_INCLUDE_EWOK_UNIFORM_BSPLINE_H_
#define EWOK_POLY_SPLINE_INCLUDE_EWOK_UNIFORM_BSPLINE_H_

#include <Eigen/Dense>

#include <ewok/polynomial.h>

#include <vector>
#include <memory>
#include <algorithm>
#include <iostream>

namespace ewok {

template<int _N, typename _Scalar = double>
class UniformBSpline {
 public:
  static const int N = _N;
  static const int DEG = N - 1;

  static const int OFFSET = N / 2 - 1;

  typedef std::shared_ptr <UniformBSpline<_N, _Scalar>> Ptr;

  typedef Eigen::Matrix<_Scalar, 1, _N> VectorNT;
  typedef Eigen::Matrix<_Scalar, _N, 1> VectorN;
  typedef Eigen::Matrix <_Scalar, _N, _N> MatrixN;
  typedef std::array<MatrixN, 4> MatrixNArray;

  explicit UniformBSpline(const _Scalar &dt) : dt_(dt) {
    double current_pow_inv_dt = 1.0;
    for (int i = 0; i < 2 * _N; ++i) {
      pow_inv_dt_[i] = current_pow_inv_dt;
      current_pow_inv_dt /= dt_;
    }
  }

  _Scalar evaluate(_Scalar t, int derivative, int & s_i) const {
    _Scalar s = t / dt_;
    s_i = s;
    _Scalar u = s - s_i;

    VectorN coefficients;
    for (int i = 0; i < _N; i++)
      coefficients[i] = control_points_[clampIdx(s_i + i - OFFSET)];

    coefficients = blending_matrix * coefficients;

    _Scalar res = 0;

    for (int i = N - 1; i >= derivative; --i) {
      res = res * u +
          coefficients[i]
              * Polynomial<_N, _Scalar>::base_coefficients_(derivative, i);
    }

    res *= pow_inv_dt_[derivative];

    return res;
  }

  _Scalar evaluate(_Scalar t, int derivative) const {
    int s_i;
    return evaluate(t, derivative, s_i);
  }

  _Scalar evaluateWithControlPointsGrad(_Scalar t,
                                        int derivative,
                                        int &grad_start_idx,
                                        VectorNT &grad) const {
    _Scalar s = t / dt_;
    int s_i = s;
    _Scalar u = s - s_i;

    VectorNT uv;
    Polynomial<_N, _Scalar>::baseCoeffsWithTime(uv, u, derivative);

    grad = pow_inv_dt_[derivative] * (uv * blending_matrix);
    grad_start_idx = s_i - OFFSET;

    VectorN coefficients;
    for (int i = 0; i < _N; i++)
      coefficients[i] = control_points_[clampIdx(s_i + i - OFFSET)];

    return grad * coefficients;
  }

  _Scalar quadraticCost(int deriv, int cp_start_idx = 0,
                        int num_points = -1) const {
    return quadraticCost(quadratic_cost_jacobian[deriv - 1]*pow_inv_dt_[2 * deriv - 1], cp_start_idx, num_points);
  }

  _Scalar quadraticCost(const MatrixN & quadratic_cost, int cp_start_idx = 0,
                        int num_points = -1) const {
    if (control_points_.size() < _N) return 0;

    if(num_points < 0) num_points = control_points_.size();

    _Scalar res = 0;

    int min_idx = std::max(minValidIdx(), cp_start_idx - (_N/2 - 1));
    int max_idx = std::min(maxValidIdx(), cp_start_idx + num_points + _N/2);

    for (int idx = min_idx; idx < max_idx; ++idx) {
      VectorN coefficients;
      for (int i = 0; i < _N; i++)
        coefficients[i] = control_points_[idx + i - OFFSET];

      _Scalar cost =
          coefficients.transpose() * quadratic_cost
              * coefficients;
      res += cost;
    }

    return res;
  }

  _Scalar quadraticCostWithGrad(int deriv,
                                std::vector <_Scalar> &cp_grad,
                                int cp_start_idx = 0,
                                int num_points = -1) const {
    return quadraticCostWithGrad(quadratic_cost_jacobian[deriv - 1]
                                     * pow_inv_dt_[2 * deriv - 1], cp_grad, cp_start_idx, num_points);
  }

  _Scalar quadraticCostWithGrad(const MatrixN & quadratic_cost,
                                std::vector <_Scalar> &cp_grad,
                                int cp_start_idx = 0,
                                int num_points = -1) const {
    if (control_points_.size() < _N) return 0;

    if(num_points < 0) num_points = control_points_.size();

    cp_grad.resize(num_points);
    std::fill(cp_grad.begin(), cp_grad.end(), 0);

    _Scalar res = 0;

    int min_idx = std::max(minValidIdx(), cp_start_idx - (_N/2 - 1));
    int max_idx = std::min(maxValidIdx(), cp_start_idx + num_points + _N/2);

    for (int idx = min_idx; idx < max_idx; ++idx) {
      VectorN coefficients;
      for (int i = 0; i < _N; i++)
        coefficients[i] = control_points_[idx + i - OFFSET];

      VectorN grad = quadratic_cost * coefficients;

      for (int i = 0; i < _N; i++) {
        int current_idx = idx + i - OFFSET;
        if(current_idx >= cp_start_idx && current_idx < (cp_start_idx+num_points)) {
          cp_grad[current_idx - cp_start_idx] += 2 * grad[i];
        }
      }

      res += coefficients.transpose() * grad;
    }
//
//    for (int i = 0; i < cp_grad.size(); i++) {
//      cp_grad[i] *= pow_inv_dt_[2 * deriv - 1];
//    }
//
//    res *= pow_inv_dt_[2 * deriv - 1];

    return res;
  }

  template<class Derived>
  void addCoeffs(
      const Eigen::MatrixBase <Derived> &coeffs_const) {
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
    for (int i = 0; i < coeffs_const.size(); ++i) {
      control_points_.push_back(coeffs_const[i]);
    }
  }

  _Scalar &coeff(int i) {
    return control_points_[i];
  }

  _Scalar coeff(int i) const {
    return control_points_[i];
  }

  inline void push_back(const _Scalar & cp) {
    control_points_.push_back(cp);
  }

  inline void setControlPointsData(const std::vector<double> & data,
                                   int cp_start_idx, int data_start_idx,
                                   int num_points) {
    for(int i=0; i<num_points; i++) {
      control_points_[cp_start_idx + i] = data[data_start_idx + i];
    }
  }

  inline void getControlPointsData(std::vector<double> & data,
                                   int cp_start_idx, int data_start_idx,
                                   int num_points) const {
    for(int i=0; i<num_points; i++) {
      data[data_start_idx + i] = control_points_[cp_start_idx + i];
    }
  }

  inline int minValidIdx() const {
    return OFFSET;
  }

  inline int maxValidIdx() const {
    return control_points_.size() - 1 - OFFSET;
  }

  inline _Scalar minValidTime() const {
    return minValidIdx() * dt_;
  }

  inline _Scalar maxValidTime() const {
    return maxValidIdx() * dt_;
  }

  inline int size() {
    return control_points_.size();
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  static const MatrixN blending_matrix;
  static const MatrixNArray quadratic_cost_jacobian;

 protected:
  inline int clampIdx(int idx) const {
    int idx1 = std::min(idx, static_cast<int>(control_points_.size()) - 1);
    return std::max(idx1, 0);
  }

  _Scalar dt_;

  std::vector <_Scalar> control_points_;

  std::array<_Scalar, 2 * _N> pow_inv_dt_;
};

uint64_t C_n_k(uint64_t n, uint64_t k) {
  if (k > n) {
    return 0;
  }
  uint64_t r = 1;
  for (uint64_t d = 1; d <= k; ++d) {
    r *= n--;
    r /= d;
  }
  return r;
}

template<int _N, typename _Scalar>
typename UniformBSpline<_N, _Scalar>::MatrixN computeBlendingMatrix() {
  typename UniformBSpline<_N, double>::MatrixN m;
  m.setZero();

  for (int i = 0; i < _N; ++i) {
    for (int j = 0; j < _N; ++j) {
      double sum = 0;

      for (int s = j; s < _N; ++s) {
        sum += std::pow(-1.0, s - j) * C_n_k(_N, s - j)
            * std::pow(_N - s - 1.0, _N - 1.0 - i);
      }
      m(i, j) = C_n_k(_N - 1, _N - 1 - i) * sum;
    }
  }

  uint64_t factorial = 1.0;
  for (int i = 2; i < _N; ++i) {
    factorial *= i;
  }

  return (m / factorial).template cast<_Scalar>();
}

template<int _N, typename _Scalar>
typename UniformBSpline<_N,
                        _Scalar>::MatrixNArray computeQuadraticCostJacobian() {
  typename UniformBSpline<_N, _Scalar>::MatrixNArray m_array;
  typename UniformBSpline<_N, double>::MatrixN
      blending_matrix = computeBlendingMatrix<_N, double>();
  typename UniformBSpline<_N, double>::MatrixNArray
      quadratic_coefficients = computeQuadraticCoefficients<_N, double>();

  for (int i = 0; i < 4; ++i) {
    typename UniformBSpline<_N, double>::MatrixN m;
    m = 0.5 * blending_matrix.transpose() *
        quadratic_coefficients[i] * blending_matrix;

    m_array[i] = m.template cast<_Scalar>();
  }
  return m_array;
}

template<int _N, typename _Scalar>
const typename UniformBSpline<_N, _Scalar>::MatrixN
    UniformBSpline<_N, _Scalar>::blending_matrix =
    computeBlendingMatrix<_N, _Scalar>();

template<int _N, typename _Scalar>
const typename UniformBSpline<_N, _Scalar>::MatrixNArray
    UniformBSpline<_N, _Scalar>::quadratic_cost_jacobian =
    computeQuadraticCostJacobian<_N, _Scalar>();

}  // namespace ewok

#endif  // EWOK_POLY_SPLINE_INCLUDE_EWOK_UNIFORM_BSPLINE_H_
