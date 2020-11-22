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

#ifndef EWOK_POLY_SPLINE_INCLUDE_EWOK_UNIFORM_BSPLINE_3D_H_
#define EWOK_POLY_SPLINE_INCLUDE_EWOK_UNIFORM_BSPLINE_3D_H_

#include <Eigen/Dense>

#include <ewok/uniform_bspline.h>

#include <visualization_msgs/Marker.h>

#include <vector>
#include <memory>
#include <algorithm>
#include <iostream>

namespace ewok {

template<int _N, typename _Scalar = double>
class UniformBSpline3D {
 public:
  static const int N = _N;
  static const int DEG = N - 1;

  static const int OFFSET = N / 2 - 1;

  typedef std::shared_ptr <UniformBSpline3D<_N, _Scalar>> Ptr;

  typedef Eigen::Matrix<_Scalar, 3, 1> Vector3;

  typedef Eigen::Matrix<_Scalar, 1, _N> VectorNT;
  typedef Eigen::Matrix<_Scalar, _N, 1> VectorN;
  typedef Eigen::Matrix <_Scalar, _N, _N> MatrixN;
  typedef std::array<MatrixN, 4> MatrixNArray;

  explicit UniformBSpline3D(const _Scalar &dt) : dt_(dt),
                                                 splines_{UniformBSpline<_N, _Scalar>(dt),
                                                          UniformBSpline<_N, _Scalar>(dt),
                                                          UniformBSpline<_N, _Scalar>(dt)} {


  }

  Vector3 evaluate(_Scalar t, int derivative) const {
    return Vector3(splines_[0].evaluate(t, derivative),
                   splines_[1].evaluate(t, derivative),
                   splines_[2].evaluate(t, derivative));
  }

  Vector3 evaluate(_Scalar t, int derivative, int & s_i) const {
    return Vector3(splines_[0].evaluate(t, derivative),
                   splines_[1].evaluate(t, derivative),
                   splines_[2].evaluate(t, derivative, s_i));
  }


  Vector3 evaluateWithControlPointsGrad(_Scalar t, int derivative, int &grad_start_idx,
                                        VectorNT & grad) const {

    Vector3 res;

    // Gradient is the same, since it doesn't depend on control points, just on t
    res[0] = splines_[0].evaluate(t, derivative);
    res[1] = splines_[1].evaluate(t, derivative);
    res[2] = splines_[2].evaluateWithControlPointsGrad(t, derivative, grad_start_idx, grad);

    return res;

  }

  _Scalar quadraticCost(int deriv, int cp_start_idx,
                        int num_points) const {
    return splines_[0].quadraticCost(deriv, cp_start_idx, num_points) +
        splines_[1].quadraticCost(deriv, cp_start_idx, num_points) +
        splines_[2].quadraticCost(deriv, cp_start_idx, num_points);
  }

  _Scalar quadraticCost(const MatrixN & quadratic_cost, int cp_start_idx,
                        int num_points) const {
    return splines_[0].quadraticCost(quadratic_cost, cp_start_idx, num_points) +
        splines_[1].quadraticCost(quadratic_cost, cp_start_idx, num_points) +
        splines_[2].quadraticCost(quadratic_cost, cp_start_idx, num_points);
  }


  _Scalar quadraticCostWithGrad(int deriv,
                                std::vector<double> & grad,
                                int cp_start_idx,
                                int num_points) const {

    _Scalar value = 0;

    std::vector <_Scalar> cp_grad(num_points);

    for(int i=0; i<3; i++) {
      value += splines_[i].quadraticCostWithGrad(deriv, cp_grad, cp_start_idx, num_points);

      for(int j=0; j<num_points; j++) {
        grad[i*num_points + j] = cp_grad[j];
      }
    }

    return value;

  }

  _Scalar quadraticCostWithGrad(const MatrixN & quadratic_cost,
                                std::vector<double> & grad,
                                int cp_start_idx,
                                int num_points) const {

    _Scalar value = 0;

    std::vector <_Scalar> cp_grad;

    for(int i=0; i<3; i++) {
      value += splines_[i].quadraticCostWithGrad(quadratic_cost, cp_grad, cp_start_idx, num_points);

      for(int j=0; j<num_points; j++) {
        grad[i*num_points + j] = cp_grad[j];
      }
    }

    return value;

  }

  _Scalar &coeff(int dim, int i) {
    return splines_[dim].coeff(i);
  }

  _Scalar coeff(int dim, int i) const {
    return splines_[dim].coeff(i);
  }

  inline void push_back(const Vector3 & vec) {
    for (int i = 0; i < 3; ++i ) {
      splines_[i].push_back(vec[i]);
    }
  }

  inline int size() {
    return splines_[0].size();
  }

  inline Vector3 getControlPoint(int i) {
    return Vector3(splines_[0].coeff(i),
                   splines_[1].coeff(i),
                   splines_[2].coeff(i));
  }

  inline void setControlPointsData(const std::vector<double> & data,
                                   int cp_start_idx,
                                   int num_points) {
    for(int i=0; i<3; i++) {
      splines_[i].setControlPointsData(data, cp_start_idx, num_points*i, num_points);
    }
  }

  inline void getControlPointsData(std::vector<double> & data,
                                   int cp_start_idx,
                                   int num_points) const {
    for(int i=0; i<3; i++) {
      splines_[i].getControlPointsData(data, cp_start_idx, num_points*i, num_points);
    }
  }

  inline int minValidIdx() const {
    return splines_[0].minValidIdx();
  }

  inline int maxValidIdx() const {
    return splines_[0].maxValidIdx();
  }

  inline _Scalar minValidTime() const {
    return splines_[0].minValidTime();
  }

  inline _Scalar maxValidTime() const {
    return splines_[0].maxValidTime();
  }

  inline _Scalar dt() const {
    return dt_;
  }


  void getVisualizationMarker(visualization_msgs::Marker & traj_marker, const std::string & ns,
                              int id, const Eigen::Vector3d & color, int fixed_id = -_N, int num_points = 0,
                              const Eigen::Vector3d & fixed_color = Eigen::Vector3d(1,1,1), _Scalar ddt = 0.1,
                              const ros::Duration & lifetime = ros::Duration(0), _Scalar scale = 0.01) {
    traj_marker.header.frame_id = "world";
    traj_marker.ns = ns;
    traj_marker.id = id;
    traj_marker.type = visualization_msgs::Marker::LINE_STRIP;
    traj_marker.action = visualization_msgs::Marker::MODIFY;
    traj_marker.scale.x = scale;
    //traj_marker.scale.y = scale;
    //traj_marker.scale.z = scale;
    traj_marker.color.a = 1.0;

    traj_marker.pose.orientation.x = 0;
    traj_marker.pose.orientation.y = 0;
    traj_marker.pose.orientation.z = 0;
    traj_marker.pose.orientation.w = 1;

    traj_marker.lifetime = lifetime;

    traj_marker.color.r = color(0);
    traj_marker.color.g = color(1);
    traj_marker.color.b = color(2);

    std_msgs::ColorRGBA c0, c1;
    c0.r = color(0);
    c0.g = color(1);
    c0.b = color(2);
    c0.a = 1.0;

    c1.r = fixed_color(0);
    c1.g = fixed_color(1);
    c1.b = fixed_color(2);
    c1.a = 1.0;


    // Time for which optimization doesn't change the point.
    _Scalar min_fixed_t = (fixed_id - _N/2) * dt_;
    _Scalar max_fixed_t = (fixed_id + num_points + _N/2) * dt_;

    for (_Scalar time = splines_[0].minValidTime(); time < splines_[0].maxValidTime(); time += ddt) {
      Vector3 v = evaluate(time, 0);

      geometry_msgs::Point p;
      p.x = v[0];
      p.y = v[1];
      p.z = v[2];
      traj_marker.points.push_back(p);


      if(time >= min_fixed_t && time < max_fixed_t) {
        traj_marker.colors.push_back(c0);
      } else {
        traj_marker.colors.push_back(c1);
      }

    }

  }

  void getControlPointsMarker(visualization_msgs::Marker & traj_marker, const std::string & ns,
                              int id, const Eigen::Vector3d & color, int fixed_id = -N, int num_points = 0,
                              const Eigen::Vector3d & fixed_color = Eigen::Vector3d(1,1,1),
                              const ros::Duration & lifetime = ros::Duration(0), _Scalar scale = 0.1) {
    traj_marker.header.frame_id = "world";
    traj_marker.ns = ns;
    traj_marker.id = id;
    traj_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    traj_marker.action = visualization_msgs::Marker::MODIFY;
    traj_marker.scale.x = scale;
    traj_marker.scale.y = scale;
    traj_marker.scale.z = scale;
    traj_marker.color.a = 1.0;

    traj_marker.pose.orientation.x = 0;
    traj_marker.pose.orientation.y = 0;
    traj_marker.pose.orientation.z = 0;
    traj_marker.pose.orientation.w = 1;

    traj_marker.lifetime = lifetime;

    traj_marker.color.r = color(0);
    traj_marker.color.g = color(1);
    traj_marker.color.b = color(2);

    std_msgs::ColorRGBA c0, c1;

    c0.r = color(0);
    c0.g = color(1);
    c0.b = color(2);
    c0.a = 1.0;

    c1.r = fixed_color(0);
    c1.g = fixed_color(1);
    c1.b = fixed_color(2);
    c1.a = 1.0;

    int min_fixed_id = fixed_id;
    int max_fixed_id = fixed_id + num_points;

    for (int i=0; i < splines_[0].size(); i++) {
      geometry_msgs::Point p;
      p.x = splines_[0].coeff(i);
      p.y = splines_[1].coeff(i);
      p.z = splines_[2].coeff(i);
      traj_marker.points.push_back(p);

      if(i >= min_fixed_id && i < max_fixed_id) {
        traj_marker.colors.push_back(c0);
      } else {
        traj_marker.colors.push_back(c1);
      }

    }

  }


  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 protected:

  _Scalar dt_;
  UniformBSpline<_N, _Scalar> splines_[3];

};

}  // namespace ewok

#endif  // EWOK_POLY_SPLINE_INCLUDE_EWOK_UNIFORM_BSPLINE_3D_H_
