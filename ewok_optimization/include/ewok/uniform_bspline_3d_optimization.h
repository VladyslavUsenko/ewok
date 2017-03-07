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

#ifndef EWOK_OPTIMIZATION_INCLUDE_EWOK_UNIFORM_BSPLINE_3D_OPTIMIZATION_H_
#define EWOK_OPTIMIZATION_INCLUDE_EWOK_UNIFORM_BSPLINE_3D_OPTIMIZATION_H_

#include <ewok/uniform_bspline_3d.h>
#include <ewok/polynomial_trajectory_3d.h>
#include <ewok/ed_ring_buffer.h>

#include <visualization_msgs/MarkerArray.h>

#include <nlopt.hpp>

namespace ewok {

template<int _N, typename _Scalar = double>
class UniformBSpline3DOptimization {
 public:

  typedef Eigen::Matrix<_Scalar, 3, 1> Vector3;
  typedef Eigen::Matrix<_Scalar, 4, 1> Vector4;

  typedef Eigen::Matrix<_Scalar, 1, _N> VectorNT;
  typedef Eigen::Matrix<_Scalar, _N, 1> VectorN;
  typedef Eigen::Matrix <_Scalar, _N, _N> MatrixN;

  typedef std::shared_ptr<UniformBSpline3DOptimization<_N, _Scalar>> Ptr;

  UniformBSpline3DOptimization(const Vector3 &start_point, _Scalar dt) :
      spline_(dt), num_cp_opt(-1), cp_opt_start_idx(_N) {
    // Make sure initial position is static at starting point
    for (int i = 0; i < _N; i++) {
      spline_.push_back(start_point);
    }

    _Scalar enpoint_time = spline_.maxValidTime() - eps;
    for(int i=0; i<2; i++) {
      int grad_start_idx;
      spline_.evaluateWithControlPointsGrad(enpoint_time, i, grad_start_idx, endpoint_grads[i]);
    }

    setDefaultWeights();
    initTrajectoryTimeOptimization();
  }


  UniformBSpline3DOptimization(ewok::PolynomialTrajectory3D<10>::Ptr & trajectory, _Scalar dt) :
      spline_(dt), num_cp_opt(-1), cp_opt_start_idx(_N), trajectory_(trajectory) {

    Vector3 start_point = trajectory_->evaluate(0,0);

    // Make sure initial position is static at starting point
    for (int i = 0; i < _N; i++) {
      spline_.push_back(start_point);
    }

    _Scalar enpoint_time = spline_.maxValidTime() - eps;
    for(int i=0; i<2; i++) {
      int grad_start_idx;
      spline_.evaluateWithControlPointsGrad(enpoint_time, i, grad_start_idx, endpoint_grads[i]);
    }

    setDefaultWeights();
    initTrajectoryTimeOptimization();
  }

  void setNumCollisionChecksPerSegment(int n) {


    for(int k = 0; k < 5; k++) {
      segment_grads[k].resize(n);

      for (int i = 0; i < n; i++) {
        _Scalar time = spline_.minValidTime() + i * spline_.dt() / n;
        int grad_start_idx;
        spline_.evaluateWithControlPointsGrad(time,
                                              k,
                                              grad_start_idx,
                                              segment_grads[k][i]);
      }
    }

  }

  void setQuadraticErrorWeights(const Vector3 & quadratic_error_weights) {
    quadratic_cost_matrix.setZero();

    for(int deriv = 2; deriv < 5; deriv ++) {
      quadratic_cost_matrix += UniformBSpline <_N, _Scalar>::quadratic_cost_jacobian[deriv - 1]
          * std::pow(1.0/spline_.dt(), 2 * deriv - 1) * quadratic_error_weights[deriv-2];
    }

  }

  void setLimits(const Vector4 & limits) {
    for(int i=0; i<4; i++) {
      limits2_[i] = limits[i]*limits[i];
    }
  }

  void setDistanceBuffer(EuclideanDistanceRingBuffer<6>::Ptr & edrb) {
    edrb_ = edrb;
  }

  void addControlPoint(const Vector3 &point, int num = 1) {
    for (int i = 0; i < num; i++) {
      spline_.push_back(point);
    }
  }

  void setControlPointOptimizationStartIdx(int cp) {
    cp_opt_start_idx = cp;
  }

  void addLastControlPoint() {
    spline_.push_back(spline_.getControlPoint(spline_.size()-1));
    cp_opt_start_idx++;
  }

  inline Vector3 getFirstOptimizationPoint() {
    return spline_.getControlPoint(cp_opt_start_idx);
  }

  void setNumControlPointsOptimized(int n) {
    num_cp_opt = n;

    //optimizer.reset(new nlopt::opt(nlopt::LD_LBFGS, 3*num_cp_opt));
    optimizer.reset(new nlopt::opt(nlopt::LD_MMA, 3*num_cp_opt));
    //optimizer.reset(new nlopt::opt(nlopt::LN_SBPLX, 3*num_cp_opt));

    std::vector<double> lb(3*num_cp_opt, -1e10);
    optimizer->set_lower_bounds(lb);

    optimizer->set_min_objective(UniformBSpline3DOptimization::wrap, this);
    optimizer->set_xtol_rel(1e-4);
  }

  _Scalar getClosestTrajectoryTime(const Vector3 & point, _Scalar t_init) {
    target_trajectory_point_ = point;


    std::vector<double> x(1);
    x[0] = t_init;

    double minf;
    nlopt::result result = trajectory_time_optimizer->optimize(x, minf);

    return x[0];

  }

  _Scalar optimize() {

    if(trajectory_.get()) {
      _Scalar enpoint_time = spline_.maxValidTime() - spline_.minValidTime() - eps;
      endpoints[0] = trajectory_->evaluate(enpoint_time, 0);
      endpoints[1] = trajectory_->evaluate(enpoint_time, 1);
    }

    std::vector<double> x(3*num_cp_opt);

    spline_.getControlPointsData(x, cp_opt_start_idx, num_cp_opt);

    double minf;
    nlopt::result result = optimizer->optimize(x, minf);

    spline_.setControlPointsData(x, cp_opt_start_idx, num_cp_opt);

    return minf;
  }

  void setControlPointsOptimizationStartIdx(int n) {
    cp_opt_start_idx = n;
  }

  void getMarkers(visualization_msgs::MarkerArray & traj_marker,
                  const std::string & ns = "spline_opitimization_markers",
                  const Vector3 & color1 = Vector3(0,1,0),
                  const Vector3 & color2 = Vector3(0,1,1)) {

    traj_marker.markers.resize(2);

    spline_.getVisualizationMarker(traj_marker.markers[0], ns, 0, color1, cp_opt_start_idx, num_cp_opt, color2);
    spline_.getControlPointsMarker(traj_marker.markers[1], ns, 1, color1, cp_opt_start_idx, num_cp_opt, color2);
  }

  void setTargetEnpoint(const Vector3 & p) {
    endpoints[0] = p;
  }

  void setTargetEnpointVelocity(const Vector3 & v) {
    endpoints[1] = v;
  }

  // Method inteded for testing only!!!
  double getAnalyticEndpointErrorGrad(std::vector<double> &grad, int deriv) const {
    grad.resize(3*num_cp_opt);
    std::fill(grad.begin(), grad.end(), 0.0);

    return endpointError(spline_, deriv, 1.0, grad);
  }

  // Method inteded for testing only!!!
  double getNumericEndpointErrorGrad(std::vector<double> &grad, int deriv) const {

    double delta = 0.0001;

    grad.resize(3*num_cp_opt);
    std::fill(grad.begin(), grad.end(), 0.0);

    std::vector<double> tmp;

    double value = endpointError(spline_, deriv, 1.0, tmp);

    for(int i=0; i<3; i++) {
      for(int j=0; j<num_cp_opt; j++) {
        UniformBSpline3D <_N, _Scalar> current_spline = spline_;

        current_spline.coeff(i, cp_opt_start_idx + j) += delta;
        grad[i*num_cp_opt + j] = (endpointError(current_spline, deriv, 1.0, tmp) - value)/delta;
      }
    }

    return value;

  }

  // Method inteded for testing only!!!
  double getAnalyticQuadraticErrorGrad(std::vector<double> &grad) const {
    grad.resize(3*num_cp_opt);
    std::fill(grad.begin(), grad.end(), 0.0);

    return quadraticCostError(spline_, 1.0, grad);
  }

  // Method inteded for testing only!!!
  double getNumericQuadraticErrorGrad(std::vector<double> &grad) const {

    double delta = 0.0001;

    grad.resize(3*num_cp_opt);
    std::fill(grad.begin(), grad.end(), 0.0);

    std::vector<double> tmp;

    double value = quadraticCostError(spline_, 1.0, tmp);

    for(int i=0; i<3; i++) {
      for(int j=0; j<num_cp_opt; j++) {
        UniformBSpline3D <_N, _Scalar> current_spline = spline_;

        current_spline.coeff(i, cp_opt_start_idx + j) += delta;
        grad[i*num_cp_opt + j] = (quadraticCostError(current_spline, 1.0, tmp) - value)/delta;
      }
    }
    return value;
  }


  // Method inteded for testing only!!!
  double getAnalyticCollisionErrorGrad(std::vector<double> &grad) const {
    grad.resize(3*num_cp_opt);
    std::fill(grad.begin(), grad.end(), 0.0);

    return collisionError(spline_, 1.0, grad);
  }

  // Method inteded for testing only!!!
  double getNumericCollisionErrorGrad(std::vector<double> &grad) const {

    double delta = 0.0001;

    grad.resize(3*num_cp_opt);
    std::fill(grad.begin(), grad.end(), 0.0);

    std::vector<double> tmp;

    double value = collisionError(spline_, 1.0, tmp);
    for(int i=0; i<3; i++) {
      for (int j = 0; j < num_cp_opt; j++) {
        UniformBSpline3D <_N, _Scalar> current_spline = spline_;

        current_spline.coeff(i, cp_opt_start_idx + j) += delta;
        grad[i * num_cp_opt + j] =
            (collisionError(current_spline, 1.0, tmp) - value) / delta;
      }
    }

    return value;

  }

  // Method inteded for testing only!!!
  double getAnalyticSoftLimitErrorGrad(std::vector<double> &grad, int deriv) const {
    grad.resize(3*num_cp_opt);
    std::fill(grad.begin(), grad.end(), 0.0);

    return softLimitError(spline_, deriv, 1.0, grad);
  }

  // Method inteded for testing only!!!
  double getNumericSoftLimitErrorGrad(std::vector<double> &grad, int deriv) const {

    double delta = 1e-8;

    grad.resize(3*num_cp_opt);
    std::fill(grad.begin(), grad.end(), 0.0);

    std::vector<double> tmp;

    double value = softLimitError(spline_, deriv, 1.0, tmp);

    for(int i=0; i<3; i++) {
      for(int j=0; j<num_cp_opt; j++) {
        UniformBSpline3D <_N, _Scalar> current_spline = spline_;

        current_spline.coeff(i, cp_opt_start_idx + j) += delta;
        grad[i*num_cp_opt + j] = (softLimitError(current_spline, deriv, 1.0, tmp) - value)/delta;
      }
    }

    return value;

  }

  void setDistanceThreshold(_Scalar d){
    distance_threshold_ = d;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 protected:

  static double wrap(const std::vector<double> &x,
                     std::vector<double> &grad, void *data) {
    return reinterpret_cast<UniformBSpline3DOptimization*>(data)
        ->combinedError(x, grad);
  }

  static double wrapTrajectoryTime(const std::vector<double> &x,
                     std::vector<double> &grad, void *data) {
    return reinterpret_cast<UniformBSpline3DOptimization*>(data)
        ->closestTimeError(x, grad);
  }


  void initTrajectoryTimeOptimization() {
    trajectory_time_optimizer.reset(new nlopt::opt(nlopt::LD_MMA, 1));

    std::vector<double> lb(1, 0);
    trajectory_time_optimizer->set_lower_bounds(lb);

    trajectory_time_optimizer->set_min_objective(UniformBSpline3DOptimization::wrapTrajectoryTime, this);
    trajectory_time_optimizer->set_xtol_rel(1e-4);
  }

  void setDefaultWeights() {
    collision_weight = 1e5;
    endpoint_error_weights = Vector3(100, 10, 0);

    setQuadraticErrorWeights(Vector3(0.1, 0.1, 0.1));
    setNumCollisionChecksPerSegment(10);

    distance_threshold_ = 0.5;

    setLimits(Vector4(2,5,0,0));
    limits_weight_ = 1.0;


  }


  // Gets the time of the closest trajectory point using Newtons method
  _Scalar closestTimeError(const std::vector<double> &x,
                           std::vector<double> &grad) {

      Vector3 e = trajectory_->evaluate(x[0], 0) - target_trajectory_point_;
      Vector3 d = trajectory_->evaluate(x[0], 1);

      _Scalar error = e.dot(e);

      if(!grad.empty()) grad[0] = 2 * e.dot(d);

      return error;

  }



  double combinedError(const std::vector<double> &x,
                       std::vector<double> &grad) {
    UniformBSpline3D <_N, _Scalar> current_spline(spline_);
    current_spline.setControlPointsData(x, cp_opt_start_idx, num_cp_opt);

    double value = 0;
    std::fill(grad.begin(), grad.end(), 0.0);

    value += quadraticCostError(current_spline, 1.0, grad);

    for(int i=0; i<2; i++) {
      value += endpointError(current_spline, i, endpoint_error_weights[i], grad);
    }

    value += collisionError(current_spline, collision_weight, grad);

    for(int i=1; i<5; i++) {
      value += softLimitError(current_spline, i, limits_weight_, grad);
    }

    return value;
  }

  double quadraticCostError(const UniformBSpline3D <_N, _Scalar> & current_spline,
                            double lambda, std::vector<double> &grad) const {

    if(grad.empty()) {
      return lambda * current_spline.quadraticCost(quadratic_cost_matrix, cp_opt_start_idx, num_cp_opt);
    } else {

      std::vector<double> cur_grad(grad.size());

      _Scalar value = current_spline.
          quadraticCostWithGrad(quadratic_cost_matrix, cur_grad, cp_opt_start_idx, num_cp_opt);

      for(int i=0; i<grad.size(); i++) {
        grad[i] += lambda * cur_grad[i];
      }

      return lambda * value;
    }

  }


  double endpointError(const UniformBSpline3D <_N, _Scalar> & current_spline,
                            int derivative, double lambda, std::vector<double> &grad) const {



    _Scalar enpoint_time = current_spline.maxValidTime() - eps;

    int s_i;
    Vector3 point, error;
    point = current_spline.evaluate(enpoint_time, derivative, s_i);
    error = point - endpoints[derivative];

    if(!grad.empty()) {

      int grad_start_idx = s_i - (_N / 2 - 1);

      for(int i=0; i<_N; i++) {
        int current_idx = grad_start_idx + i;
        if(current_idx >= cp_opt_start_idx && current_idx < (cp_opt_start_idx+num_cp_opt)) {

          int idx = current_idx-cp_opt_start_idx;

          grad[0*num_cp_opt + idx] += lambda * 2 * error[0] * endpoint_grads[derivative][i];
          grad[1*num_cp_opt + idx] += lambda * 2 * error[1] * endpoint_grads[derivative][i];
          grad[2*num_cp_opt + idx] += lambda * 2 * error[2] * endpoint_grads[derivative][i];
        }
      }
    }

    return lambda * error.dot(error);

  }

  double collisionError(const UniformBSpline3D <_N, _Scalar> & current_spline,
                        double lambda, std::vector<double> &grad) const {

    if(!edrb_.get()) return 0;

    double total_error = 0;

    int start_segment_idx = cp_opt_start_idx - (_N/2 - 1);
    int end_segment_idx = std::min(cp_opt_start_idx + num_cp_opt + _N/2, spline_.maxValidIdx());

    for(int segment_idx = start_segment_idx; segment_idx < end_segment_idx; segment_idx++) {
      for(int k=0; k<segment_grads[0].size(); k++) {
        _Scalar current_time = (segment_idx + k/ static_cast<double>(segment_grads[0].size())) * spline_.dt();

        int s_i;
        Vector3 point, grad_p;
        point = current_spline.evaluate(current_time, 0, s_i);

        _Scalar dist = edrb_->getDistanceWithGrad(point, grad_p);

        if(dist > distance_threshold_) continue;

        _Scalar diff = dist - distance_threshold_;
        _Scalar error = 0.5 * diff * diff / distance_threshold_;
        total_error += error;

        //ROS_INFO_STREAM("dist: " << dist << " error: " << error);

        if (!grad.empty()) {

          int grad_start_idx = s_i - (_N / 2 - 1);

          for (int i = 0; i < _N; i++) {
            int current_idx = grad_start_idx + i;
            if (current_idx >= cp_opt_start_idx
                && current_idx < (cp_opt_start_idx + num_cp_opt)) {

              int idx = current_idx - cp_opt_start_idx;

              grad[0 * num_cp_opt + idx] +=
                  lambda * (diff/distance_threshold_) * grad_p[0] * segment_grads[0][k][i];
              grad[1 * num_cp_opt + idx] +=
                  lambda * (diff/distance_threshold_) * grad_p[1] * segment_grads[0][k][i];
              grad[2 * num_cp_opt + idx] +=
                  lambda * (diff/distance_threshold_) * grad_p[2] * segment_grads[0][k][i];
            }
          }
        }
      }
    }

    //if(total_error > 0) ROS_INFO_STREAM("Total collision error: " << lambda * total_error);
    return lambda * total_error;

  }


  double softLimitError(const UniformBSpline3D <_N, _Scalar> & current_spline, int derivative,
                        double lambda, std::vector<double> &grad) const {

    double total_error = 0;

    //std::cerr << "limits2_ " << limits2_.transpose() << std::endl;

    if(limits2_[derivative-1] == 0) return 0;

    int start_segment_idx = cp_opt_start_idx - (_N/2 - 1);
    int end_segment_idx = std::min(cp_opt_start_idx + num_cp_opt + _N/2, spline_.maxValidIdx());

    for(int segment_idx = start_segment_idx; segment_idx < end_segment_idx; segment_idx++) {
      for(int k=0; k<segment_grads[derivative].size(); k++) {
        _Scalar current_time = (segment_idx + k/ static_cast<double>(segment_grads[derivative].size())) * spline_.dt();

        int s_i;
        Vector3 point;
        point = current_spline.evaluate(current_time, derivative, s_i);

        _Scalar norm2 = point.dot(point);

        if(norm2 <= limits2_[derivative-1]) continue;

        _Scalar diff = norm2 - limits2_[derivative-1];

        _Scalar error = std::exp(diff);
        total_error += (error - 1.0);

        //std::cerr << "diff: " << diff << " error: " << (error - 1.0) << " norm2: " << norm2 << std::endl;

        if (!grad.empty()) {

          int grad_start_idx = s_i - (_N / 2 - 1);

          for (int i = 0; i < _N; i++) {
            int current_idx = grad_start_idx + i;
            if (current_idx >= cp_opt_start_idx
                && current_idx < (cp_opt_start_idx + num_cp_opt)) {

              int idx = current_idx - cp_opt_start_idx;

              grad[0 * num_cp_opt + idx] +=
                  lambda * 2 * point[0] * error * segment_grads[derivative][k][i];
              grad[1 * num_cp_opt + idx] +=
                  lambda * 2 * point[1] * error * segment_grads[derivative][k][i];
              grad[2 * num_cp_opt + idx] +=
                  lambda * 2 * point[2] * error * segment_grads[derivative][k][i];
            }
          }
        }
      }
    }

    //if(total_error > 0) ROS_INFO_STREAM("Total collision error: " << lambda * total_error);
    return lambda * total_error;

  }


  const double eps = 1e-4;

  _Scalar distance_threshold_;

  Vector3 endpoints[2];
  VectorNT endpoint_grads[2];

  std::vector<VectorNT, Eigen::aligned_allocator<VectorNT>> segment_grads[5];

  Vector4 limits2_;
  _Scalar limits_weight_;

  Vector3 endpoint_error_weights;
  _Scalar collision_weight;

  typename UniformBSpline3D <_N, _Scalar>::MatrixN quadratic_cost_matrix;

  UniformBSpline3D <_N, _Scalar> spline_;

  int num_cp_opt;
  int cp_opt_start_idx;

  std::shared_ptr<nlopt::opt> optimizer, trajectory_time_optimizer;

  EuclideanDistanceRingBuffer<6>::Ptr edrb_;

  ewok::PolynomialTrajectory3D<10>::Ptr trajectory_;

  Vector3 target_trajectory_point_;

};

}  // namespace ewok


#endif  // EWOK_OPTIMIZATION_INCLUDE_EWOK_UNIFORM_BSPLINE_3D_OPTIMIZATION_H_
