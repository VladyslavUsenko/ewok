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

#include <chrono>

#include <ros/ros.h>

#include <ewok/uniform_bspline_3d_optimization.h>
#include <ewok/polynomial_3d_optimization.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "spline_optimization_example");
  ros::NodeHandle nh;

  ROS_INFO("Started spline_optimization_example");

  ros::Publisher global_traj_pub = nh.advertise<visualization_msgs::MarkerArray>(
      "global_trajectory",
      1,
      true);

  ros::Publisher before_opt_pub = nh.advertise<visualization_msgs::MarkerArray>(
      "before_optimization",
      1,
      true);
  ros::Publisher after_opt_pub = nh.advertise<visualization_msgs::MarkerArray>(
      "after_optimization",
      1,
      true);

  ros::Publisher occ_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/occupied", 1, true);
  ros::Publisher free_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/free", 1, true);
  ros::Publisher dist_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/distance", 1, true);

  // Set up global trajectory
  const Eigen::Vector4d limits(1.2, 4, 0, 0);

  ewok::Polynomial3DOptimization<10> po(limits*0.8);

  typename ewok::Polynomial3DOptimization<10>::Vector3Array vec;
  vec.push_back(Eigen::Vector3d(-5,-5, 1));
  vec.push_back(Eigen::Vector3d(5, -2.5, 1));
  vec.push_back(Eigen::Vector3d(-5, 2.5, 1));
  vec.push_back(Eigen::Vector3d( 5, 5, 1));

  auto traj = po.computeTrajectory(vec);

  visualization_msgs::MarkerArray traj_marker;
  traj->getVisualizationMarkerArray(traj_marker, "trajectory", Eigen::Vector3d(1, 0, 0));

  global_traj_pub.publish(traj_marker);

  // Set up collision buffer
  ewok::EuclideanDistanceRingBuffer<6>::Ptr edrb(new ewok::EuclideanDistanceRingBuffer<6>(0.15, 1));
  ewok::EuclideanDistanceRingBuffer<6>::PointCloud cloud;

  for(float z = -2; z < 2; z += 0.05) {
    cloud.push_back(Eigen::Vector4f(0, 0.1, z, 0));
  }

  edrb->insertPointCloud(cloud, Eigen::Vector3f(0,0,0));
  edrb->insertPointCloud(cloud, Eigen::Vector3f(0,0,0));

  edrb->updateDistance();

  visualization_msgs::Marker m_occ, m_free, m_dist;
  edrb->getMarkerOccupied(m_occ);
  edrb->getMarkerFree(m_free);
  edrb->getMarkerDistance(m_dist, 0.5);

  occ_marker_pub.publish(m_occ);
  free_marker_pub.publish(m_free);
  dist_marker_pub.publish(m_dist);

  // Set up spline optimization
  const int num_points = 7;
  const double dt = 0.5;

  Eigen::Vector3d start_point(-5, -5, 0), end_point(5, 5, 0);
  ewok::UniformBSpline3DOptimization<6> spline_opt(traj, dt);

  for (int i = 0; i < num_points; i++) {
    spline_opt.addControlPoint(vec[0]);
  }

  spline_opt.setNumControlPointsOptimized(num_points);
  spline_opt.setDistanceBuffer(edrb);
  spline_opt.setLimits(limits);


  double tc = spline_opt.getClosestTrajectoryTime(Eigen::Vector3d(-3, -5, 1), 2.0);
  ROS_INFO_STREAM("Closest time: " << tc);

  ROS_INFO("Finished setting up data");

  double current_time = 0;

  double total_opt_time = 0;
  int num_iterations = 0;

  ros::Rate r(1.0/dt);
  while (ros::ok() && current_time < traj->duration()) {
    r.sleep();
    current_time += dt;

    visualization_msgs::MarkerArray before_opt_markers, after_opt_markers;

    spline_opt.getMarkers(before_opt_markers, "before_opt",
                          Eigen::Vector3d(1, 0, 0),
                          Eigen::Vector3d(1, 0, 0));

    auto t1 = std::chrono::high_resolution_clock::now();
    double error = spline_opt.optimize();
    auto t2 = std::chrono::high_resolution_clock::now();

    double miliseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count() / 1.0e6;

    total_opt_time += miliseconds;
    num_iterations++;


    ROS_INFO_STREAM("Finished optimization in " << miliseconds << " ms. Error: " << error);

    spline_opt.getMarkers(after_opt_markers, "after_opt",
                          Eigen::Vector3d(0, 1, 0),
                          Eigen::Vector3d(0, 1, 1));

    after_opt_pub.publish(after_opt_markers);

    spline_opt.addLastControlPoint();

    ros::spinOnce();
  }

  ROS_INFO_STREAM("Mean optimization time " << total_opt_time/num_iterations);

  return 0;
}