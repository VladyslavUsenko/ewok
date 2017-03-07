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

#include <ewok/polynomial.h>
#include <gtest/gtest.h>

template <int _N, typename _Scalar>
struct TypeDefinitions {
  static const int N = _N;
  typedef _Scalar Scalar;
};

typedef ::testing::Types <
                          TypeDefinitions<2,double>,
                          TypeDefinitions<4,double>,
                          TypeDefinitions<6,double>,
                          TypeDefinitions<8,double>,
                          TypeDefinitions<10,double>,
                          TypeDefinitions<12,double>,
                          TypeDefinitions<14,double>,
                          TypeDefinitions<16,double>,
                          TypeDefinitions<2,float>,
                          TypeDefinitions<4,float>,
                          TypeDefinitions<6,float>,
                          TypeDefinitions<8,float>,
                          TypeDefinitions<10,float>,
                          TypeDefinitions<12,float>,
                          TypeDefinitions<14,float>,
                          TypeDefinitions<16,float>
                          > Implementations;



template <class T>
class PolynomialTest : public testing::Test {
};


TYPED_TEST_CASE(PolynomialTest, Implementations);

// Tests endpoint constraint matrix according to formulas (18)-(20) in [1].
// See Polynomial class.
TYPED_TEST(PolynomialTest, TestEndpointConstraintMatrix)
{
  typedef ewok::Polynomial<TypeParam::N, typename TypeParam::Scalar>
      PolynomialType;

  typename PolynomialType::MatrixN m, m_test;

  std::vector<typename TypeParam::Scalar> times = {0.1, 0.5, 1.0, 2.0, 5.0, 10.0};

  for(auto t: times) {
    PolynomialType::endpointConstrainsMatrix(m, t);

    m_test.setZero();

    for(int i=0; i<TypeParam::N/2; i++) {
      double prod = 1.0;
      for(int k=1; k<=i; k++) prod *= k;
      m_test(i, i) = prod;
    }

    for(int i=0; i<TypeParam::N/2; i++) {
      for(int j=0; j<TypeParam::N; j++) {
        if(i <= j) {
          double prod = 1.0;
          for(int k=j; k>j-i; k--) prod *= k;
          m_test(TypeParam::N/2 + i, j) = prod * pow(t, j-i);
        }
      }
    }

    for(int i=0; i<m.rows(); i++) {
      for(int j=0; j<m.cols(); j++) {
        ASSERT_PRED_FORMAT2(::testing::internal::CmpHelperFloatingPointEQ<typename TypeParam::Scalar>, m(i,j), m_test(i,j));
      }
    }
  }
}


// Given random endpoint constraints and segment times
// checks if computed polynomial coefficients follow constraints
// Note: Skips the cases when enpointConstraintMatrix is not invertable.
TYPED_TEST(PolynomialTest, TestEndpointConstraints)
{
  typedef ewok::Polynomial<TypeParam::N, typename TypeParam::Scalar>
      PolynomialType;

  typename PolynomialType::MatrixN m, m_inv;

  const double max_inversion_error = 1e-5;

  std::vector<typename TypeParam::Scalar> times = {0.1, 0.5, 1.0, 2.0, 5.0, 10.0};

  for(auto t: times) {

      typename PolynomialType::VectorN endpoint_constraints, coeffs, endpoint_test_vector;

      endpoint_constraints.setRandom();

      PolynomialType::endpointConstrainsMatrix(m, t);

      coeffs = m.fullPivLu().solve(endpoint_constraints);

      double relative_error = (m*coeffs - endpoint_constraints).norm() /
          endpoint_constraints.norm();

      if(relative_error < max_inversion_error) {

        PolynomialType p(coeffs);

        for(int i=0; i<TypeParam::N/2; i++) {
            endpoint_test_vector[i] = p.evaluate(0, i);
            endpoint_test_vector[TypeParam::N/2+i] = p.evaluate(t, i);
        }

        for(int i=0; i<TypeParam::N; i++) {

          const double eps = 1e-4;

          // Checks error in relation to the value;
          double eps1 = std::abs(endpoint_constraints[i]) * eps;
          ASSERT_NEAR(endpoint_constraints[i], endpoint_test_vector[i], eps1)
              << "Error at location: " << i
              << "\ne: " << endpoint_constraints.transpose()
              << "\net: " << endpoint_test_vector.transpose();

        }

      } else {
        std::cerr << "For time " << t << ": The relative error for inversion is: " << relative_error << " skipping test." << std::endl;
      }
  }
}


// Check if analytic time derivatives (velocity, acceleration ...)
// are close to numeric derivatives
TYPED_TEST(PolynomialTest, TestTimeDerivativesNumeric)
{
  typedef ewok::Polynomial<TypeParam::N, typename TypeParam::Scalar>
      PolynomialType;

  typename PolynomialType::MatrixN m, m_inv;

  const double eps = 1e-5;
  const int num_checks = 10;

  double max_error;

  if(std::is_same<typename TypeParam::Scalar, double>::value) {
    max_error = 1e-6;
  } else {
    // Somehow numeric error for floating point is quiet large...
    max_error = 1;
  }

  std::vector<typename TypeParam::Scalar> times = {0.1, 0.5, 1.0, 2.0, 5.0, 10.0};

  for(auto t: times) {

    typename PolynomialType::VectorN coeffs;

    coeffs.setRandom();

    PolynomialType p(coeffs);

    for(int i=0; i<=num_checks; ++i) {
      double current_time = i*t/num_checks;
      for(int deriv=0; deriv<5; deriv++) {
        double time_deriv_analytic = p.evaluate(current_time, deriv+1);

        double time_deriv_numeric = (p.evaluate(current_time + eps, deriv)
            - p.evaluate(current_time-eps, deriv))/(2*eps);


        double relative_max_error = std::abs(time_deriv_analytic) * max_error;
        ASSERT_NEAR(time_deriv_analytic, time_deriv_numeric, relative_max_error)
          << "Time: " << t << " derivative: " << deriv;

      }
    }

  }
}


// Check if integral over quadratic cost is close to numeric evaluation
TYPED_TEST(PolynomialTest, TestTimeIntegralNumeric)
{
  typedef ewok::Polynomial<TypeParam::N, typename TypeParam::Scalar>
      PolynomialType;

  typename PolynomialType::MatrixN m, m_inv;

  const int num_intervals = 100;

  const double max_error = 0.01;

  std::vector<typename TypeParam::Scalar> times = {0.1, 0.5, 1.0, 2.0, 5.0, 10.0};

  for(auto t: times) {

    typename PolynomialType::VectorN coeffs;
    coeffs.setRandom();
    PolynomialType p(coeffs);

    double dt = t/num_intervals;

    for(int deriv=1; deriv<5; deriv++) {

      auto Q = PolynomialType::quadraticCostJacobian(t, deriv);
      double integral_analytic = 0.5 * coeffs.transpose() * Q * coeffs;
      double integral_numeric = 0;

      for(int i=0; i<num_intervals; ++i) {
        double current_time = (i + 0.5) * t / num_intervals;
        double val = p.evaluate(current_time, deriv);
        integral_numeric += val*val*dt;
      }
      double relative_max_error = std::abs(integral_analytic) * max_error;
      ASSERT_NEAR(integral_analytic, integral_numeric, relative_max_error)
        << "Time: " << t << " derivative: " << deriv;
    }
  }
}


int main(int argc, char **argv) {
  //srand((unsigned int) time(0));
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
