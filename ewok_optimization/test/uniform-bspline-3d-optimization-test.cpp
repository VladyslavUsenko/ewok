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

#include <ewok/uniform_bspline_3d_optimization.h>
#include <gtest/gtest.h>

#include <iostream>
#include <iomanip>

template <int _N, typename _Scalar>
struct TypeDefinitions {
  static const int N = _N;
  typedef _Scalar Scalar;
};

typedef ::testing::Types <
                          TypeDefinitions<6,double>
                          > Implementations;



template <class T>
class UniformBSpline3DOptimizationTest : public testing::Test {
};


TYPED_TEST_CASE(UniformBSpline3DOptimizationTest, Implementations);


TYPED_TEST(UniformBSpline3DOptimizationTest, TestEndpointDerivatives)
{
  typedef ewok::UniformBSpline3DOptimization<TypeParam::N, typename TypeParam::Scalar>
      UniformBSpline3DOptimizationType;

  typedef typename UniformBSpline3DOptimizationType::Vector3 Vector3;

  const double max_error = 1e-4;
  const int num_points = 7;

  Eigen::Vector3d start_point(-5, -5, 0);
  UniformBSpline3DOptimizationType spline_opt(start_point, 0.5);

  for (int i = 0; i < num_points; i++) {
    spline_opt.addControlPoint(UniformBSpline3DOptimizationType::Vector3::Random() * 5);
  }

  spline_opt.setTargetEnpoint(Vector3(0,0,0));
  spline_opt.setTargetEnpointVelocity(Vector3(1,0,0));

  spline_opt.setNumControlPointsOptimized(num_points);

  for(int deriv=0; deriv<2; deriv++) {
    std::vector<double> numeric_grad, analytic_grad;
    double numeric_val, analytic_val;

    numeric_val = spline_opt.getNumericEndpointErrorGrad(numeric_grad, deriv);
    analytic_val = spline_opt.getAnalyticEndpointErrorGrad(analytic_grad, deriv);

    ASSERT_NEAR(numeric_val, analytic_val, max_error);

    std::cerr << "Analytic grad:\t";
    for(int i=0; i<analytic_grad.size(); i++) {
      std::cerr << analytic_grad[i] << "\t";
    }
    std::cerr << std::endl;

    std::cerr << "Numeric grad:\t";
    for(int i=0; i<numeric_grad.size(); i++) {
      std::cerr << numeric_grad[i] << "\t";
    }
    std::cerr << std::endl;

    for(int i=0; i<analytic_grad.size(); i++) {
      ASSERT_NEAR(numeric_grad[i], analytic_grad[i], max_error);
    }
  }
}

TYPED_TEST(UniformBSpline3DOptimizationTest, TestQuadraticDerivatives)
{
  typedef ewok::UniformBSpline3DOptimization<TypeParam::N, typename TypeParam::Scalar>
      UniformBSpline3DOptimizationType;

  typedef typename UniformBSpline3DOptimizationType::Vector3 Vector3;

  const double max_error = 1e-2;
  const int num_points = 7;

  Eigen::Vector3d start_point(-5, -5, 0);
  UniformBSpline3DOptimizationType spline_opt(start_point, 0.5);

  for (int i = 0; i < num_points; i++) {
    spline_opt.addControlPoint(UniformBSpline3DOptimizationType::Vector3::Random() * 5);
  }

  spline_opt.setTargetEnpoint(Vector3(0,0,0));
  spline_opt.setTargetEnpointVelocity(Vector3(1,0,0));

  spline_opt.setNumControlPointsOptimized(num_points);

  for(int it=1; it<4; it++) {

    Vector3 quadratic_error_weights = Vector3::Random().array().abs();

    spline_opt.setQuadraticErrorWeights(quadratic_error_weights);

    std::vector<double> numeric_grad, analytic_grad;
    double numeric_val, analytic_val;

    numeric_val = spline_opt.getNumericQuadraticErrorGrad(numeric_grad);
    analytic_val = spline_opt.getAnalyticQuadraticErrorGrad(analytic_grad);

    ASSERT_NEAR(numeric_val, analytic_val, max_error);

    std::cerr << std::fixed << std::setprecision(10) << "A grad:\t";
    for(int i=0; i<analytic_grad.size(); i++) {
      std::cerr << analytic_grad[i] << "\t";
    }
    std::cerr << std::endl;

    std::cerr << "N grad:\t";
    for(int i=0; i<numeric_grad.size(); i++) {
      std::cerr << numeric_grad[i] << "\t";
    }
    std::cerr << std::endl;

    for(int i=0; i<analytic_grad.size(); i++) {
      double relative_max_error = std::max(max_error, std::abs(analytic_grad[i])*max_error);
      ASSERT_NEAR(numeric_grad[i], analytic_grad[i], relative_max_error)
         << " i: " << i;
    }
  }
}

TYPED_TEST(UniformBSpline3DOptimizationTest, TestCollisionDerivatives)
{
  typedef ewok::UniformBSpline3DOptimization<TypeParam::N, typename TypeParam::Scalar>
      UniformBSpline3DOptimizationType;

  typedef typename UniformBSpline3DOptimizationType::Vector3 Vector3;

  const double max_error = 1e-3;
  const int num_points = 7;

  Eigen::Vector3d start_point(-5, -5, 0);
  UniformBSpline3DOptimizationType spline_opt(start_point, 0.5);

  for(int i=0; i<5; i++) {
    spline_opt.addControlPoint(start_point);
  }

  for (int i = 0; i < num_points; i++) {
    spline_opt.addControlPoint(start_point + Vector3(1,1,0)*(i+1));
  }

  spline_opt.setTargetEnpoint(Vector3(0,0,0));
  spline_opt.setTargetEnpointVelocity(Vector3(1,0,0));

  spline_opt.setNumControlPointsOptimized(num_points);
  spline_opt.setControlPointOptimizationStartIdx(11);


  // Set up collision buffer
  ewok::EuclideanDistanceRingBuffer<6>::Ptr edrb(new ewok::EuclideanDistanceRingBuffer<6>(0.15, 1));
  ewok::EuclideanDistanceRingBuffer<6>::PointCloud cloud;

  for(float z = -2; z < 2; z += 0.05) {
  cloud.push_back(Eigen::Vector4f(0, 0.2, z, 0));
  }

  edrb->insertPointCloud(cloud, Eigen::Vector3f(0,0,0));
  edrb->insertPointCloud(cloud, Eigen::Vector3f(0,0,0));

  edrb->updateDistance();

  spline_opt.setDistanceBuffer(edrb);


  std::vector<double> numeric_grad, analytic_grad;
  double numeric_val, analytic_val;

  numeric_val = spline_opt.getNumericCollisionErrorGrad(numeric_grad);
  analytic_val = spline_opt.getAnalyticCollisionErrorGrad(analytic_grad);

  ASSERT_NEAR(numeric_val, analytic_val, max_error);

  std::cerr << std::fixed << std::setprecision(10) << "A grad:\t";
  for(int i=0; i<analytic_grad.size(); i++) {
    std::cerr << analytic_grad[i] << "\t";
  }
  std::cerr << std::endl;

  std::cerr << "N grad:\t";
  for(int i=0; i<numeric_grad.size(); i++) {
    std::cerr << numeric_grad[i] << "\t";
  }
  std::cerr << std::endl;

  for(int i=0; i<analytic_grad.size(); i++) {
    double relative_max_error = std::max(max_error, std::abs(analytic_grad[i])*max_error);
    ASSERT_NEAR(numeric_grad[i], analytic_grad[i], relative_max_error) << " i: " << i;
  }

}



TYPED_TEST(UniformBSpline3DOptimizationTest, TestSoftLimitDerivatives)
{
  typedef ewok::UniformBSpline3DOptimization<TypeParam::N, typename TypeParam::Scalar>
      UniformBSpline3DOptimizationType;

  typedef typename UniformBSpline3DOptimizationType::Vector3 Vector3;
  typedef typename UniformBSpline3DOptimizationType::Vector4 Vector4;

  const double max_error = 0.01;
  const int num_points = 7;

  Eigen::Vector3d start_point(-5, -5, 0), end_point(5,5,0);
  UniformBSpline3DOptimizationType spline_opt(start_point, 0.5);

  for (int i = 0; i < num_points; i++) {
    spline_opt.addControlPoint(start_point + 0.8 * i*(end_point-start_point)/num_points);
  }

  spline_opt.setTargetEnpoint(end_point);
  spline_opt.setTargetEnpointVelocity(Vector3(0,0,0));

  spline_opt.setLimits(Vector4(2,3,0,0));

  spline_opt.setNumControlPointsOptimized(num_points);

  for(int deriv=1; deriv<5; deriv++) {
    std::vector<double> numeric_grad, analytic_grad;
    double numeric_val, analytic_val;

    numeric_val = spline_opt.getNumericSoftLimitErrorGrad(numeric_grad, deriv);
    analytic_val = spline_opt.getAnalyticSoftLimitErrorGrad(analytic_grad, deriv);

    ASSERT_NEAR(numeric_val, analytic_val, max_error) << "derivative: " << deriv;

    std::cerr << "Analytic grad:\t";
    for(int i=0; i<analytic_grad.size(); i++) {
      std::cerr << analytic_grad[i] << "\t";
    }
    std::cerr << std::endl;

    std::cerr << "Numeric grad:\t";
    for(int i=0; i<numeric_grad.size(); i++) {
      std::cerr << numeric_grad[i] << "\t";
    }
    std::cerr << std::endl;

    for(int i=0; i<analytic_grad.size(); i++) {
      double relative_max_error = std::max(max_error, std::abs(analytic_grad[i])*max_error);
      ASSERT_NEAR(numeric_grad[i], analytic_grad[i], relative_max_error) << "derivative: " << deriv;
    }
  }
}


int main(int argc, char **argv) {
  //srand((unsigned int) time(0));
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
