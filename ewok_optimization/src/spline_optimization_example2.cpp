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

int main(int argc, char **argv) {

  srand((unsigned int) time(0));

  ros::init(argc, argv, "spline_optimization_example");
  ros::NodeHandle nh;

  ROS_INFO("Started spline_optimization_example");

  ros::Publisher before_opt_pub = nh.advertise<visualization_msgs::MarkerArray>(
      "before_optimization",
      1,
      true);
  ros::Publisher after_opt_pub = nh.advertise<visualization_msgs::MarkerArray>(
      "after_optimization",
      1,
      true);

  const int num_points = 7;

  Eigen::Vector3d start_point(-5, -5, 0), end_point(5, 5, 0);
  ewok::UniformBSpline3DOptimization<6> spline_opt(start_point, 0.5);

  for (int i = 0; i < num_points; i++) {
    spline_opt.addControlPoint(Eigen::Vector3d::Random() * 5);
  }

  //spline_opt.addControlPoint(end_point, 6);

  spline_opt.setTargetEnpoint(Eigen::Vector3d(0,0,0));
  spline_opt.setTargetEnpointVelocity(Eigen::Vector3d(1,0,0));

  spline_opt.setNumControlPointsOptimized(num_points);

  ROS_INFO("Finished setting up data");

  visualization_msgs::MarkerArray before_opt_markers, after_opt_markers;

  spline_opt.getMarkers(before_opt_markers, "before_opt",
                        Eigen::Vector3d(1, 0, 0),
                        Eigen::Vector3d(1, 0, 0));

  auto t1 = std::chrono::high_resolution_clock::now();
  double error = spline_opt.optimize();
  auto t2 = std::chrono::high_resolution_clock::now();

  double miliseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count() / 1.0e6;

  ROS_INFO_STREAM("Finished optimization in " << miliseconds << " ms. Error: " << error);

  spline_opt.getMarkers(after_opt_markers, "after_opt",
                        Eigen::Vector3d(0, 1, 0),
                        Eigen::Vector3d(0, 0, 1));

  before_opt_pub.publish(before_opt_markers);
  after_opt_pub.publish(after_opt_markers);

  ros::spin();

  return 0;
}