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

#include <ewok/uniform_bspline.h>
#include <gtest/gtest.h>

template <int _N, typename _Scalar>
struct TypeDefinitions {
  static const int N = _N;
  typedef _Scalar Scalar;
};

typedef ::testing::Types <
                          TypeDefinitions<4,double>,
                          TypeDefinitions<6,double>,
                          TypeDefinitions<8,double>,
                          TypeDefinitions<10,double>
                          //TypeDefinitions<4,float>,
                          //TypeDefinitions<6,float>,
                          //TypeDefinitions<8,float>,
                          //TypeDefinitions<10,float>
                          > Implementations;



template <class T>
class UniformBSplineTest : public testing::Test {
};


TYPED_TEST_CASE(UniformBSplineTest, Implementations);


// Check if analytic time derivatives (velocity, acceleration ...)
// are close to numeric derivatives
TYPED_TEST(UniformBSplineTest, TestTimeDerivativesNumeric)
{
  typedef ewok::UniformBSpline<TypeParam::N, typename TypeParam::Scalar>
      UniformBSplineType;

  const double eps = 1e-6;

  double max_error;

  max_error = 1e-4;


  std::vector<typename TypeParam::Scalar> times =
      {0.1, 0.5, 1.0, 2.0, 5.0, 10.0};

  for(auto t: times) {

    Eigen::VectorXd coeffs = Eigen::VectorXd::LinSpaced(TypeParam::N*2, -5, 20)
        + Eigen::VectorXd::Random(TypeParam::N*2)*3;

    UniformBSplineType ubs(t);
    ubs.addCoeffs(coeffs);

    for(double ct = ubs.minValidTime(); ct < ubs.maxValidTime(); ct += t/10.0) {

      int max_deriv_to_check = std::min(TypeParam::N-2, 4);

      for(int deriv = 0; deriv < max_deriv_to_check; ++deriv) {
        double time_deriv_analytic = ubs.evaluate(ct, deriv+1);
        double time_deriv_numeric = (ubs.evaluate(ct+eps, deriv) - ubs.evaluate(ct-eps, deriv))/(2*eps);

        double relative_max_error = std::abs(time_deriv_analytic) * max_error;
        ASSERT_NEAR(time_deriv_analytic, time_deriv_numeric, relative_max_error)
        << "Time: " << t << " derivative: " << deriv << " current_time " << ct;

      }
    }
  }
}


// Check if spline is continuous
TYPED_TEST(UniformBSplineTest, TestSplineContinuity)
{
  typedef ewok::UniformBSpline<TypeParam::N, typename TypeParam::Scalar>
      UniformBSplineType;

  const double eps = 1e-8;

  double max_error;

  max_error = 1e-4;


  std::vector<typename TypeParam::Scalar> times =
      {0.1, 0.5, 1.0, 2.0, 5.0, 10.0};

  for(auto t: times) {

    Eigen::VectorXd coeffs = Eigen::VectorXd::LinSpaced(TypeParam::N*2, -5, 20)
        + Eigen::VectorXd::Random(TypeParam::N*2)*3;

    UniformBSplineType ubs(t);
    ubs.addCoeffs(coeffs);

    for(int idx = ubs.minValidIdx()+1; idx < ubs.maxValidIdx(); ++idx) {

      double ct = idx * t;

      int max_deriv_to_check = std::min(TypeParam::N-2, 4);

      for(int deriv = 0; deriv < max_deriv_to_check; ++deriv) {

        double val = ubs.evaluate(ct, deriv);
        double val_m_eps = ubs.evaluate(ct-eps, deriv);

        double relative_max_error = std::abs(val) * max_error;
        ASSERT_NEAR(val, val_m_eps, relative_max_error)
        << "Time: " << t << " derivative: " << deriv << " current_time " << ct;


      }
    }
  }
}


// Check if integral over quadratic cost is close to numeric evaluation
TYPED_TEST(UniformBSplineTest, TestTimeIntegralNumeric)
{
    typedef ewok::UniformBSpline<TypeParam::N, typename TypeParam::Scalar>
        UniformBSplineType;

    const int num_intervals = 100;
    const double max_error = 0.0001;

    std::vector<typename TypeParam::Scalar> times = {0.1, 0.5, 1.0, 2.0, 5.0, 10.0};

    for(auto t: times) {

      Eigen::VectorXd coeffs = Eigen::VectorXd::LinSpaced(TypeParam::N*2, -5, 20)
          + Eigen::VectorXd::Random(TypeParam::N*2)*3;

      UniformBSplineType ubs(t);
      ubs.addCoeffs(coeffs);

      double dt = t/num_intervals;


      for(int deriv=1; deriv < 5; deriv++) {

        double integral_analytic = ubs.quadraticCost(deriv);
        double integral_numeric = 0;

        //std::cerr << "ubs.minValidTime() " << ubs.minValidTime() << " ubs.maxValidTime() " << ubs.maxValidTime() << std::endl;

        for(double current_time = ubs.minValidTime() + dt/2;
              current_time < ubs.maxValidTime(); current_time += dt) {
          double val = ubs.evaluate(current_time, deriv);
          integral_numeric += val*val*dt;
        }

        double relative_max_error = std::abs(integral_analytic) * max_error;
        ASSERT_NEAR(integral_analytic, integral_numeric, relative_max_error)
          << "Time: " << t << " derivative: " << deriv;
      }
    }
}


// Check if analytic time derivatives (velocity, acceleration ...)
// are close to numeric derivatives
TYPED_TEST(UniformBSplineTest, TestControlPointDerivativesNumeric)
{
  typedef ewok::UniformBSpline<TypeParam::N, typename TypeParam::Scalar>
      UniformBSplineType;

  const double eps = 1e-6;
  double max_error = 1e-4;


  std::vector<typename TypeParam::Scalar> times =
      {0.1, 0.5, 1.0, 2.0, 5.0, 10.0};

  for(auto t: times) {

    Eigen::VectorXd coeffs = Eigen::VectorXd::LinSpaced(TypeParam::N*2, -5, 20)
        + Eigen::VectorXd::Random(TypeParam::N*2)*3;

    UniformBSplineType ubs(t);
    ubs.addCoeffs(coeffs);

    for(double ct = ubs.minValidTime(); ct < ubs.maxValidTime(); ct += t/10.0) {

      int max_deriv_to_check = std::min(TypeParam::N-2, 4);

      for(int deriv = 0; deriv < max_deriv_to_check; ++deriv) {

        int grad_start_idx;
        typename UniformBSplineType::VectorNT analytic_grad, numeric_grad;
        double val = ubs.evaluateWithControlPointsGrad(ct, deriv, grad_start_idx, analytic_grad);
        double val1 = ubs.evaluate(ct, deriv);

        {
          double relative_max_error = std::abs(val) * max_error;
          ASSERT_NEAR(val, val1, relative_max_error);
        }

        for(int i=0; i<TypeParam::N; i++) {
          UniformBSplineType ubs1 = ubs;
          ubs1.coeff(i+grad_start_idx) += eps;
          numeric_grad[i] = (ubs1.evaluate(ct, deriv) - val)/eps;
        }


        for(int i=0; i<TypeParam::N; i++) {
          double relative_max_error = std::abs(analytic_grad[i]) * max_error;
          relative_max_error = std::max(relative_max_error, max_error);

          ASSERT_NEAR(analytic_grad[i], numeric_grad[i], relative_max_error)
          << "Time: " << t << " derivative: " << deriv
          << " current_time " << ct << " i "
          << i << " grad_start_idx " << grad_start_idx
          << " val " << val
          << " val1 " << val1
          << "\nanalytic_grad: " << analytic_grad
          << "\nnumeric_grad: " << numeric_grad;
        }
      }
    }
  }
}


// Check if integral over quadratic cost is close to numeric evaluation
TYPED_TEST(UniformBSplineTest, TestQuadraticCostControlPointDerivatives)
{
  typedef ewok::UniformBSpline<TypeParam::N, typename TypeParam::Scalar>
      UniformBSplineType;

  const double eps = 1e-8;
  double max_error = 0.01;

  std::vector<typename TypeParam::Scalar> times = {0.1, 0.5, 1.0, 2.0, 5.0, 10.0};

  for(auto t: times) {

    Eigen::VectorXd coeffs = Eigen::VectorXd::LinSpaced(TypeParam::N*2, -5, 20)
        + Eigen::VectorXd::Random(TypeParam::N*2)*3;

    UniformBSplineType ubs(t);
    ubs.addCoeffs(coeffs);

    int cp_opt_start_idx = TypeParam::N;
    int num_opt_points = 2*TypeParam::N - cp_opt_start_idx;

    for(int deriv=1; deriv < 5; deriv++) {

    std::vector<typename TypeParam::Scalar> analytic_grad, numeric_grad;
    double val = ubs.quadraticCostWithGrad(deriv, analytic_grad, cp_opt_start_idx, num_opt_points);
    double val1 = ubs.quadraticCost(deriv, cp_opt_start_idx, num_opt_points);

    {
      double relative_max_error = std::abs(val) * max_error;
      ASSERT_NEAR(val, val1, relative_max_error)
        << "Time: " << t << " derivative: " << deriv;
    }

    numeric_grad.resize(analytic_grad.size());

    for(int i=0; i<numeric_grad.size(); i++) {
      UniformBSplineType ubs1 = ubs;
      ubs1.coeff(cp_opt_start_idx + i) += eps;

      numeric_grad[i] = (ubs1.quadraticCost(deriv, cp_opt_start_idx, num_opt_points) - val) / eps;
    }

    for(int i=0; i<numeric_grad.size(); i++) {
      double relative_max_error = std::abs(analytic_grad[i]) * max_error;
      relative_max_error = std::max(relative_max_error, max_error);

      ASSERT_NEAR(analytic_grad[i], numeric_grad[i], relative_max_error)
        << "Time: " << t << " derivative: " << deriv << " i " << i;
    }

    }
  }
}

int main(int argc, char **argv) {
  //srand((unsigned int) time(0));
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
