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

#ifndef EWOK_OPTIMIZATION_INCLUDE_EWOK_POLYNOMIAL_3D_OPTIMIZATION_H_
#define EWOK_OPTIMIZATION_INCLUDE_EWOK_POLYNOMIAL_3D_OPTIMIZATION_H_

#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <Eigen/IterativeLinearSolvers>
#include <Eigen/CholmodSupport>

#include <ewok/polynomial_trajectory_3d.h>

namespace ewok {

template<int N, typename _Scalar = double>
class Polynomial3DOptimization {
 public:

  typedef std::shared_ptr <Polynomial3DOptimization> Ptr;

  typedef Eigen::Matrix<_Scalar, 2, 1> Vector2;
  typedef Eigen::Matrix<_Scalar, 3, 1> Vector3;
  typedef Eigen::Matrix<_Scalar, 4, 1> Vector4;

  typedef Eigen::Matrix <_Scalar, N, N> MatrixN;
  typedef Eigen::Matrix<_Scalar, N, 1> VectorN;
  typedef Eigen::Matrix<_Scalar, 2 * N, 1> Vector2N;
  typedef Eigen::Matrix<_Scalar, 1, N> VectorNT;

  typedef Eigen::Matrix<_Scalar, Eigen::Dynamic, 1> VectorX;
  typedef Eigen::Matrix <_Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixX;
  typedef Eigen::Matrix <_Scalar, N, Eigen::Dynamic> MatrixNX;

  typedef Eigen::SparseMatrix <_Scalar> SparseMatrix;
  typedef Eigen::Triplet <_Scalar> Triplet;

  typedef std::vector <MatrixN, Eigen::aligned_allocator<MatrixN>>
      MatrixNVector;

  typedef std::vector <Vector3, Eigen::aligned_allocator<Vector3>>
      Vector3Array;

  typedef PolynomialTrajectory3D <N, _Scalar> PolTraj;

  typedef typename PolynomialTrajectory3D<N, _Scalar>::Ptr PolTrajPtr;

  typedef Polynomial <N, _Scalar> Pol;
  typedef PolynomialSegment3D <N, _Scalar> PolSeg;

  typedef typename Polynomial<N, _Scalar>::Ptr PolPtr;
  typedef typename PolynomialSegment3D<N, _Scalar>::Ptr PolSegPtr;

  //typedef Eigen::SimplicialLLT<SparseMatrix> SymSolver;
  //typedef Eigen::ConjugateGradient<SparseMatrix, Eigen::Lower | Eigen::Upper> SymSolver;
  typedef Eigen::CholmodDecomposition <SparseMatrix> SymSolver;

  MatrixNX x_coeffs;
  MatrixNX y_coeffs;
  MatrixNX z_coeffs;

  int num_segments;
  int num_waypoints;

  _Scalar min_segment_time;

  Vector4 limits;

  Vector3 quadratic_error_weights;

  Polynomial3DOptimization(const Vector4 &limits) {

    if (N % 2 != 0 || N < 10) {
      std::cerr
          << "TrajectoryOptimization supports only N that is Even and >=10"
          << std::endl;
      return;
    }

    min_segment_time = 0.1;

    // Velocity, Acceleration, Jerk, Snap
    // If 0 - no constraints
    this->limits = limits.array().abs();

    if (this->limits(0) == 0) {
      std::cerr << "Velocity should not be 0" << std::endl;
      return;
    }

    quadratic_error_weights = Vector3::Ones();

  }

  inline void setQuadraticErrorWeights(const Vector3 & weights) {
    quadratic_error_weights = weights;
  }


  PolTrajPtr computeTrajectory(const Vector3Array &waypoints) {

    num_waypoints = waypoints.size();
    num_segments = waypoints.size() - 1;

    VectorX traj_segment_times = VectorX(num_segments);

    for (int i = 0; i < num_segments; i++) {
      _Scalar dist = (waypoints[i]
          - waypoints[i + 1]).norm();

      _Scalar translational_time = dist / limits(0);
      traj_segment_times[i] = std::max(translational_time, min_segment_time);

    }

    std::cerr << "Trajectory times: " << traj_segment_times.transpose() << std::endl;

    return computeTrajectoryWithTimes(waypoints, traj_segment_times);

  }

  PolTrajPtr computeTrajectoryWithTimes(const Vector3Array &filtered_waypoints,
                                        const VectorX & traj_segment_times) {

    num_waypoints = filtered_waypoints.size();
    num_segments = filtered_waypoints.size() - 1;

    VectorX x_points(num_waypoints), y_points(num_waypoints),
        z_points(num_waypoints), yaw_points(num_waypoints);

    for (int i = 0; i < num_waypoints; i++) {
      x_points[i] = filtered_waypoints[i][0];
      y_points[i] = filtered_waypoints[i][1];
      z_points[i] = filtered_waypoints[i][2];
    }

    // Compute M
    SparseMatrix M_inv_f, M_inv_p;
    compute_M(M_inv_f, M_inv_p);


    if (num_segments > 1) {
      // Compute R
      SparseMatrix Rpp, Rpf;
      MatrixNVector A_inv_vector, Q_vector;
      compute_Rpp_Rpf(M_inv_f, M_inv_p, traj_segment_times, Rpp, Rpf, A_inv_vector, Q_vector);

      // Compute inverse of Rpp
      SymSolver ch_solver;
      ch_solver.compute(Rpp);

      compute_coeffs(ch_solver, Rpf, A_inv_vector, x_points, x_coeffs);
      compute_coeffs(ch_solver, Rpf, A_inv_vector, y_points, y_coeffs);
      compute_coeffs(ch_solver, Rpf, A_inv_vector, z_points, z_coeffs);

//      for(int i=0; i<num_segments; i++) {
//        _Scalar x_error = x_coeffs.col(i).transpose() * Q_vector[i] * x_coeffs.col(i);
//        _Scalar y_error = y_coeffs.col(i).transpose() * Q_vector[i] * y_coeffs.col(i);
//        _Scalar z_error = z_coeffs.col(i).transpose() * Q_vector[i] * z_coeffs.col(i);
//
//        std::cerr << "Segment: " << i << std::endl;
//        std::cerr << "x_error: " << x_error << std::endl;
//        std::cerr << "y_error: " << y_error << std::endl;
//        std::cerr << "z_error: " << z_error << std::endl;
//        std::cerr << "error: " << x_error + y_error + z_error << std::endl;
//        std::cerr << "time: " << traj_segment_times[i] << std::endl;
//        std::cerr << "Q:\n" << Q_vector[i] << std::endl;
//      }


    } else {
      compute_coeffs_one_segment(x_points, traj_segment_times, x_coeffs);
      compute_coeffs_one_segment(y_points, traj_segment_times, y_coeffs);
      compute_coeffs_one_segment(z_points, traj_segment_times, z_coeffs);

    }

    PolTrajPtr traj(new PolTraj);
    for (int i = 0; i < num_segments; i++) {
      PolSegPtr seg(new PolSeg(traj_segment_times[i], x_coeffs.col(i), y_coeffs.col(i), z_coeffs.col(i)));
      traj->addSegment(seg);
    }

    return traj;

  }

  /*
  bool check_limits(VectorX &updated_traj_segment_times) {
    updated_traj_segment_times.resize(traj_segment_times.size());

    float dt = 0.05;
    float limit_scalar = 1.1;

    bool traj_passed = true;

    for (int segment = 0; segment < num_segments; segment++) {
      Vector4 max_value;
      max_value.setZero();

      _Scalar max_yaw_rate = 0;

      VectorN x_c = x_coeffs.col(segment);
      VectorN y_c = y_coeffs.col(segment);
      VectorN z_c = z_coeffs.col(segment);

      for (float t = 0; t < traj_segment_times[segment]; t += dt) {
        for (int d = 0; d < 4; d++) {
          VectorNT pol = Polynomial<N>::baseCoeffsWithTime(t, 1 + d);
          Vector3 val(pol * x_c, pol * y_c, pol * z_c);

          max_value(d) = std::max(max_value(d), val.norm());
        }
      }

      for (int i = 0; i < 4; i++) {
        if (limits(i) > 0 && limits(i) < max_value(i)) {
          traj_passed = false;
        }
      }


      _Scalar time_scalar = limit_scalar * max_value(0) / limits(0);
      for (int i = 0; i < 3; i++) {
        if (limits(i + 1) > 0) {
          time_scalar = std::max(time_scalar,
                                 limit_scalar * max_value(i + 1)
                                     / limits(i + 1));
        }
      }

      updated_traj_segment_times[segment] =
          traj_segment_times[segment] * time_scalar;
      updated_traj_segment_times[segment] =
          std::max(updated_traj_segment_times[segment], min_segment_time);

      std::cout << "Segment " << segment << ": " << max_value.transpose()
                << " time_scalar " << time_scalar << std::endl;
    }

    std::cout << "updated_traj_segment_times "
              << updated_traj_segment_times.transpose() << std::endl;

    return traj_passed;

  }
*/




  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:

  void compute_Rpp_Rpf(const SparseMatrix &M_inv_f,
                       const SparseMatrix &M_inv_p,
                       const VectorX & traj_segment_times,
                       SparseMatrix &Rpp,
                       SparseMatrix &Rpf,
                       MatrixNVector &A_inv_vector,
                       MatrixNVector &Q_vector) {

    SparseMatrix AQA(num_segments * N, num_segments * N);
    std::vector <Triplet> AQA_triplets;

    A_inv_vector.resize(num_segments);
    Q_vector.resize(num_segments);

    MatrixN A, A_inv_T_Q_A_inv;
    for (int i = 0; i < num_segments; i++) {

      Q_vector[i].setZero();
      for(int j=0; j<3; j++) {
        MatrixN Q;
        Polynomial<N>::quadraticCostJacobian(Q,traj_segment_times[i],
                                                 2+j);

        Q_vector[i] += Q * quadratic_error_weights[j];
      }

      //std::cout << "Q\n" << Q << std::endl;

      Polynomial<N>::endpointConstrainsMatrix(A, traj_segment_times[i]);
      //std::cout << "A\n" << A << std::endl;

      A_inv_vector[i] = A.lu().inverse();
      //std::cout << "A_inv\n" << A_inv << std::endl;

      A_inv_T_Q_A_inv =
          A_inv_vector[i].transpose() * Q_vector[i] * A_inv_vector[i];
      //std::cout << "A_inv_T_Q_A_Tv\n" << A_inv_T_Q_A_T << std::endl;

      for (int x = 0; x < N; x++) {
        for (int y = 0; y < N; y++) {
          AQA_triplets.push_back(Triplet(i * N + x,
                                         i * N + y,
                                         A_inv_T_Q_A_inv(x, y)));
        }
      }

    }

    AQA.setFromTriplets(AQA_triplets.begin(), AQA_triplets.end());

    SparseMatrix M_inv_p_T = M_inv_p.transpose();

    Rpp = M_inv_p_T * AQA * M_inv_p;
    Rpf = M_inv_p_T * AQA * M_inv_f;

    //std::cout << "Rpp\n" << MatrixX(Rpp) << std::endl;

  }

  void compute_derivative_vector(const VectorX &waypoints,
                                 const VectorX &bp,
                                 int segment_idx,
                                 VectorN &v) {

    int higher_deriv_start_idx = 4 * (num_waypoints - 2);
    int num_high_deriv = (N / 2 - 5);
    int num_intermediate_waypoints = (num_waypoints - 2);

    v.setZero();
    v(0) = waypoints[segment_idx];
    v(N / 2) = waypoints[segment_idx + 1];

    if (segment_idx == 0) {
      for (int i = 0; i < 4; i++) {
        v(N / 2 + 1 + i) = bp[i];
      }

      for (int i = 0; i < num_high_deriv; i++) {
        v(N / 2 + 5 + i) = bp[higher_deriv_start_idx + i];
      }

    } else if (segment_idx == num_waypoints - 2) {

      int start_idx = 4 * (num_waypoints - 3);
      for (int i = 0; i < 4; i++) {
        v(1 + i) = bp[start_idx + i];
      }

      int higher_start_idx = higher_deriv_start_idx
          + num_high_deriv * (2 * num_intermediate_waypoints - 1);
      for (int i = 0; i < num_high_deriv; i++) {
        v(5 + i) = bp[higher_start_idx + i];
      }
    } else {

      int start_idx = 4 * (segment_idx - 1);
      for (int i = 0; i < 4; i++) {
        v(1 + i) = bp[start_idx + i];
        v(N / 2 + 1 + i) = bp[start_idx + 4 + i];
      }

      int higher_start_idx =
          higher_deriv_start_idx + num_high_deriv * (2 * (segment_idx - 1) + 1);
      for (int i = 0; i < num_high_deriv; i++) {
        v(5 + i) = bp[higher_start_idx + i];
      }
      for (int i = 0; i < num_high_deriv; i++) {
        v(N / 2 + 5 + i) = bp[higher_start_idx + num_high_deriv + i];
      }

    }
  }

  void compute_coeffs(const SymSolver &ch_solver,
                      const SparseMatrix &Rpf,
                      const MatrixNVector &A_inv_vector,
                      const VectorX &waypoints, MatrixNX &coeffs) {

    VectorX bf(Rpf.cols());
    bf.setZero();
    bf.head(num_waypoints) = waypoints;

    VectorX Rfp_T_bf = Rpf * bf;

    VectorX bp = -ch_solver.solve(Rfp_T_bf);

    coeffs.resize(N, num_segments);
    coeffs.setZero();

    {
      int higher_deriv_start_idx = 4 * (num_waypoints - 2);
      int num_higher_deriv = 2 * (N / 2 - 5);

      VectorN v;
      compute_derivative_vector(waypoints, bp, 0, v);
      coeffs.col(0) = A_inv_vector[0] * v;

      int start_idx = 4 * (num_waypoints - 3);
      compute_derivative_vector(waypoints, bp, num_segments - 1, v);
      coeffs.col(num_segments - 1) = A_inv_vector[num_segments - 1] * v;

      for (int i = 1; i < num_segments - 1; i++) {
        compute_derivative_vector(waypoints, bp, i, v);
        coeffs.col(i) = A_inv_vector[i] * v;
      }

    }

  }

  void compute_coeffs_one_segment(const VectorX &waypoints, const VectorX & traj_segment_times, MatrixNX &coeffs) {

    VectorX bf = waypoints;

    coeffs.resize(N, num_segments);
    coeffs.setZero();

    {
      MatrixN A, A_inv;
      Polynomial<N>::endpointConstrainsMatrix(A, traj_segment_times[0]);
      A_inv = A.lu().inverse();

      VectorN v;
      v.setZero();
      v(0) = waypoints[0];
      v(N / 2) = waypoints[1];

      coeffs.col(0) = A_inv * v;
    }

  }

  /*
  void compute_coeffs_two_states(const VectorN &v, MatrixNX &coeffs) {

    coeffs.resize(N, num_segments);
    coeffs.setZero();

    {
      MatrixN A, A_inv;
      Polynomial<N>::computeA(A, traj_segment_times[0]);
      A_inv = A.lu().inverse();

      coeffs.col(0) = A_inv * v;
    }

  }
  */

  _Scalar compute_function_value(const MatrixNVector &Q_vector,
                                 const MatrixNX &coeffs) {
    _Scalar error_sum = 0;
    for (int i = 0; i < num_segments; i++) {
      VectorN p = coeffs.col(i);
      _Scalar error = p.transpose() * Q_vector[i] * p;
      error_sum += error;

//			std::cout << "====================================" << std::endl;
//			std::cout << "i: " << i << std::endl;
//			std::cout << "error: " << error << std::endl;
//			std::cout << "error1: " << error/traj_segment_times[i] << std::endl;
//			std::cout << "error2: " << sqrt(error)/traj_segment_times[i] << std::endl;
//			std::cout << "====================================" << std::endl;


    }

    return error_sum;

  }

  static _Scalar compute_Q_element(int i,
                                   int l,
                                   int derivative,
                                   const VectorX &t) {
    if (i < derivative || l < derivative) {
      return _Scalar(0);
    } else {
      _Scalar prod = 1;
      for (int m = 0; m < derivative; m++) {
        prod *= (i - m) * (l - m);
      }
      int denom = i + l - 2 * derivative + 1;

      return 2 * t(denom) * prod / denom;
    }
  }

  void compute_M(SparseMatrix &M_inv_f, SparseMatrix &M_inv_p) {

    SparseMatrix M(num_segments * N, num_segments * N);
    {
      std::vector <Triplet> M_triplets;

      int line_number = 0;
      // Set position values of the waypoints
      for (int i = 0; i < num_segments; i++) {
        M_triplets.push_back(Triplet(line_number, N * i, 1));
        line_number++;
      }
      M_triplets.push_back(Triplet(line_number, N * num_segments - N / 2, 1));
      line_number++;

      // derivatives in the beginning and in the end equals to 0
      for (int d = 1; d < N / 2; d++) {
        M_triplets.push_back(Triplet(line_number,
                                     N * (num_segments - 1) + N / 2 + d,
                                     1));
        line_number++;
        M_triplets.push_back(Triplet(line_number, d, 1));
        line_number++;
      }

      // Set equality constraints on segment boundaries
      for (int i = 0; i < num_segments - 1; i++) {
        for (int d = 0; d < 5; d++) {
          M_triplets.push_back(Triplet(line_number, N * i + N / 2 + d, 1));
          M_triplets.push_back(Triplet(line_number, N * (i + 1) + d, -1));
          line_number++;
        }
      }

      // Unknown derivatives of order <=4 (Should be the same for left and right segments)
      for (int i = 0; i < num_segments - 1; i++) {
        for (int d = 1; d < 5; d++) {
          M_triplets.push_back(Triplet(line_number, N * i + N / 2 + d, 1));
          line_number++;
        }
      }

      // Unknown derivatives of order >= 5 (First left derivatives and then right derivatives)
      for (int i = 0; i < num_segments - 1; i++) {
        for (int d = 5; d < N / 2; d++) {
          M_triplets.push_back(Triplet(line_number, N * i + N / 2 + d, 1));
          line_number++;
        }
        for (int d = 5; d < N / 2; d++) {
          M_triplets.push_back(Triplet(line_number, N * (i + 1) + d, 1));
          line_number++;
        }
      }

      assert(line_number == num_segments * N);

      M.setFromTriplets(M_triplets.begin(), M_triplets.end());

    }

    SparseMatrix M_inv(num_segments * N, num_segments * N),
        I(num_segments * N, num_segments * N);
    I.setIdentity();

    Eigen::SparseLU <SparseMatrix> solver;
    solver.analyzePattern(M);
    solver.factorize(M);
    M_inv = solver.solve(I);

    int num_free_derivatives = (4 + 2 * (N / 2 - 5)) * (num_segments - 1);
    M_inv_f = M_inv.leftCols(num_segments * N - num_free_derivatives);
    M_inv_p = M_inv.rightCols(num_free_derivatives);

//		std::cout << "M: (non-zero: " << M.nonZeros() << " )\n" << MatrixX(M) << std::endl;
//		std::cout << "M_inv: (non-zero: " << M_inv.nonZeros() << " )\n" << MatrixX(M_inv) << std::endl;
//		SparseMatrix MM_inv = M*M_inv;
//		std::cout << "MM_inv: (non-zero: " << MM_inv.nonZeros() << " )\n" << MatrixX(MM_inv) << std::endl;

  }

};

}

#endif /* TRAJECTORY_H_ */
