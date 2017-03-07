/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef BSPLINE_LEE_POSITION_CONTROLLER_NODE_H
#define BSPLINE_LEE_POSITION_CONTROLLER_NODE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include "rotors_control/common.h"
#include "rotors_control/lee_position_controller.h"

#include "ewok/uniform_bspline_3d.h"

namespace rotors_control {

class BSplineLeePositionControllerNode {
 public:
    BSplineLeePositionControllerNode(ros::NodeHandle & nh);
  ~BSplineLeePositionControllerNode();

  void InitializeParams();

 private:

  LeePositionController lee_position_controller_;

  std::string namespace_;

  // subscribers
  ros::Subscriber cmd_poly_sub_;
  ros::Subscriber odometry_sub_;

  ros::Publisher motor_velocity_reference_pub_;

  ros::NodeHandle nh;

  double dt;
  ewok::UniformBSpline3D<6, double>::Ptr b_spline_;
  ros::Time init_time;
  double last_yaw;

  void PointCallback(
      const geometry_msgs::PointConstPtr & point_msg);

  void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);

  void getTrajectoryPoint(double t,
            mav_msgs::EigenTrajectoryPoint& command_trajectory, bool & yaw_from_traj);


  };
}

#endif // BSPLINE_LEE_POSITION_CONTROLLER_NODE_H
