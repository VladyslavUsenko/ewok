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

#ifndef EWOK_POLY_SPLINE_INCLUDE_EWOK_POLYNOMIAL_SEGMENT_3D_H_
#define EWOK_POLY_SPLINE_INCLUDE_EWOK_POLYNOMIAL_SEGMENT_3D_H_

#include <ewok/polynomial.h>

#include <visualization_msgs/Marker.h>

namespace ewok {

template<int _N, typename _Scalar = double>
class PolynomialSegment3D {
 public:

  typedef Polynomial <_N, _Scalar> Pol;

  typedef Eigen::Matrix<_Scalar, 3, 1> Vector3;

  typedef std::shared_ptr <PolynomialSegment3D<_N, _Scalar>> Ptr;

  template<class Derived>
  PolynomialSegment3D(_Scalar segment_time,
                      const Eigen::MatrixBase <Derived> &x_coeffs,
                      const Eigen::MatrixBase <Derived> &y_coeffs,
                      const Eigen::MatrixBase <Derived> &z_coeffs):
      segment_time_(segment_time),
      polynomials_{Pol (x_coeffs),
                   Pol (y_coeffs),
                   Pol (z_coeffs)} {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, _N);
  }

  _Scalar duration() const {
    return segment_time_;
  }

  Vector3 evaluate(const _Scalar time, int derivative = 0) const {
    _Scalar t = clampTime(time);
    return Vector3(polynomials_[0].evaluate(t, derivative),
                   polynomials_[1].evaluate(t, derivative),
                   polynomials_[2].evaluate(t, derivative));
  }

  template<class Derived>
  void getDerivatives(_Scalar t,
                      const Eigen::MatrixBase <Derived> &x_const,
                      const Eigen::MatrixBase <Derived> &y_const,
                      const Eigen::MatrixBase <Derived> &z_const,
                      const Eigen::MatrixBase <Derived> &yaw_const) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, _N / 2);

    Eigen::MatrixBase <Derived> &x =
        const_cast<Eigen::MatrixBase <Derived> &>(x_const);

    Eigen::MatrixBase <Derived> &y =
        const_cast<Eigen::MatrixBase <Derived> &>(y_const);

    Eigen::MatrixBase <Derived> &z =
        const_cast<Eigen::MatrixBase <Derived> &>(z_const);

    t = clampTime(t);

    for (int i = 0; i < _N / 2; i++) {
      x[i] = polynomials_[0]->evaluate(t, i);
      y[i] = polynomials_[1]->evaluate(t, i);
      z[i] = polynomials_[2]->evaluate(t, i);
    }
  }

  void getVisualizationMarker(visualization_msgs::Marker &traj_marker,
                              const std::string &ns,
                              int id,
                              const Eigen::Vector3d &color,
                              _Scalar dt = 0.1,
                              const ros::Duration &lifetime = ros::Duration(0),
                              _Scalar scale = 0.01) {
    traj_marker.header.frame_id = "world";
    traj_marker.ns = ns;
    traj_marker.id = id;
    traj_marker.type = visualization_msgs::Marker::LINE_STRIP;
    traj_marker.action = visualization_msgs::Marker::MODIFY;
    traj_marker.scale.x = scale;
    traj_marker.scale.y = scale;
    traj_marker.scale.z = scale;
    traj_marker.color.a = 1.0;

    traj_marker.lifetime = lifetime;

    traj_marker.color.r = color(0);
    traj_marker.color.g = color(1);
    traj_marker.color.b = color(2);

    for (_Scalar time = 0; time < segment_time_; time += dt) {
      geometry_msgs::Point p;
      p.x = polynomials_[0].evaluate(time, 0);
      p.y = polynomials_[1].evaluate(time, 0);
      p.z = polynomials_[2].evaluate(time, 0);
      traj_marker.points.push_back(p);
    }

    {
      geometry_msgs::Point p;
      p.x = polynomials_[0].evaluate(segment_time_, 0);
      p.y = polynomials_[1].evaluate(segment_time_, 0);
      p.z = polynomials_[2].evaluate(segment_time_, 0);
      traj_marker.points.push_back(p);
    }

  }

 private:

  inline _Scalar clampTime(_Scalar t) const {
    if (t > segment_time_)
      return segment_time_;

    if (t < 0)
      return 0;

    return t;
  }

  _Scalar segment_time_;
  Pol polynomials_[3];

};

}

#endif //  EWOK_POLY_SPLINE_INCLUDE_EWOK_POLYNOMIAL_SEGMENT_3D_H_
