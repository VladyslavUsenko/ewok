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

#include <ewok/polynomial_3d_optimization.h>

int main(int argc, char **argv) {

  srand((unsigned int) time(0));

  ros::init(argc, argv, "polynomial_optimization_example");
  ros::NodeHandle nh;

  ROS_INFO("Started polynomial_optimization_example");

  ros::Publisher before_opt_pub = nh.advertise<visualization_msgs::MarkerArray>(
      "after_optimization",
      1,
      true);

  const int num_points = 7;

  Eigen::Vector3d start_point(-5, -5, 1), middle_point(2,0,1), end_point(5, 5, 1);

  const Eigen::Vector4d limits(2,5,0,0);

  ewok::Polynomial3DOptimization<10> po(limits);

  typename ewok::Polynomial3DOptimization<10>::Vector3Array vec;
  vec.push_back(start_point);
  vec.push_back(middle_point);
  vec.push_back(end_point);

  auto traj = po.computeTrajectory(vec);

  visualization_msgs::MarkerArray traj_marker;
  traj->getVisualizationMarkerArray(traj_marker, "trajectory", Eigen::Vector3d(1, 0, 0));

  before_opt_pub.publish(traj_marker);

  ros::spin();

  return 0;
}