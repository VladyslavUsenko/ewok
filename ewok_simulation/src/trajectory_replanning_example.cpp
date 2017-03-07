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



#include <thread>
#include <chrono>
#include <map>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <visualization_msgs/MarkerArray.h>

#include <ewok/polynomial_3d_optimization.h>
#include <ewok/uniform_bspline_3d_optimization.h>


const int POW = 6;

double dt ;
int num_opt_points;

bool initialized = false;

std::ofstream f_time, opt_time;


ewok::PolynomialTrajectory3D<10>::Ptr traj;
ewok::EuclideanDistanceRingBuffer<POW>::Ptr edrb;
ewok::UniformBSpline3DOptimization<6>::Ptr spline_optimization;

ros::Publisher occ_marker_pub, free_marker_pub, dist_marker_pub, trajectory_pub, current_traj_pub;
tf::TransformListener * listener;


void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    //ROS_INFO("recieved depth image");

    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    const float fx = 554.254691191187;
    const float fy = 554.254691191187;
    const float cx = 320.5;
    const float cy = 240.5;

    tf::StampedTransform transform;


    try{

        listener->lookupTransform("world", msg->header.frame_id,
                                  msg->header.stamp, transform);
    }
    catch (tf::TransformException &ex) {
        ROS_INFO("Couldn't get transform");
        ROS_WARN("%s",ex.what());
        return;
    }


    Eigen::Affine3d dT_w_c;
    tf::transformTFToEigen(transform, dT_w_c);

    Eigen::Affine3f T_w_c = dT_w_c.cast<float>();

    float * data = (float *) cv_ptr->image.data;


    auto t1 = std::chrono::high_resolution_clock::now();

    ewok::EuclideanDistanceRingBuffer<POW>::PointCloud cloud1;

    for(int u=0; u < cv_ptr->image.cols; u+=4) {
        for(int v=0; v < cv_ptr->image.rows; v+=4) {
            float val = data[v*cv_ptr->image.cols + u];

            //ROS_INFO_STREAM(val);

            if(std::isfinite(val)) {
                Eigen::Vector4f p;
                p[0] = val*(u - cx)/fx;
                p[1] = val*(v - cy)/fy;
                p[2] = val;
                p[3] = 1;

                p = T_w_c * p;

                cloud1.push_back(p);
            }
        }
    }

    Eigen::Vector3f origin = (T_w_c * Eigen::Vector4f(0,0,0,1)).head<3>();

    auto t2 = std::chrono::high_resolution_clock::now();

    if(!initialized) {
        Eigen::Vector3i idx;
        edrb->getIdx(origin, idx);

        ROS_INFO_STREAM("Origin: " << origin.transpose() << " idx " << idx.transpose());

        edrb->setOffset(idx);

        initialized = true;
    } else {
        Eigen::Vector3i origin_idx, offset, diff;
        edrb->getIdx(origin, origin_idx);

        offset = edrb->getVolumeCenter();
        diff = origin_idx - offset;

        while(diff.array().any()) {
            //ROS_INFO("Moving Volume");
            edrb->moveVolume(diff);

            offset = edrb->getVolumeCenter();
            diff = origin_idx - offset;
        }


    }

    //ROS_INFO_STREAM("cloud1 size: " << cloud1.size());


    auto t3 = std::chrono::high_resolution_clock::now();

    edrb->insertPointCloud(cloud1, origin);

    auto t4 = std::chrono::high_resolution_clock::now();

    f_time << std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count() << " " <<
              std::chrono::duration_cast<std::chrono::nanoseconds>(t3-t2).count() << " " <<
              std::chrono::duration_cast<std::chrono::nanoseconds>(t4-t3).count() << std::endl;

    visualization_msgs::Marker m_occ, m_free;
    edrb->getMarkerOccupied(m_occ);
    edrb->getMarkerFree(m_free);


    occ_marker_pub.publish(m_occ);
    free_marker_pub.publish(m_free);

}

void sendCommandCallback(const ros::TimerEvent& e) {

    auto t1 = std::chrono::high_resolution_clock::now();

    edrb->updateDistance();

    visualization_msgs::MarkerArray traj_marker;

    auto t2 = std::chrono::high_resolution_clock::now();
    spline_optimization->optimize();
    auto t3 = std::chrono::high_resolution_clock::now();

    opt_time << std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count() << " "
        << std::chrono::duration_cast<std::chrono::nanoseconds>(t3-t2).count() << std::endl;

    Eigen::Vector3d pc = spline_optimization->getFirstOptimizationPoint();

    geometry_msgs::Point pp;
    pp.x = pc[0];
    pp.y = pc[1];
    pp.z = pc[2];

    trajectory_pub.publish(pp);

    spline_optimization->getMarkers(traj_marker);
    current_traj_pub.publish(traj_marker);

    spline_optimization->addLastControlPoint();

    visualization_msgs::Marker m_dist;
    edrb->getMarkerDistance(m_dist, 0.5);
    dist_marker_pub.publish(m_dist);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "trajectory_replanning_example");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string path = ros::package::getPath("ewok_simulation") + "/benchmarking/";

    ROS_INFO_STREAM("path: " << path);

    f_time.open(path + "mapping_time.txt");
    opt_time.open(path + "optimization_time.txt");


    listener = new tf::TransformListener;

    occ_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/occupied", 5);
    free_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/free", 5);
    dist_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/distance", 5);

    trajectory_pub =
            nh.advertise<geometry_msgs::Point>(
                    "command/point", 10);

    ros::Publisher traj_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("global_trajectory", 1, true);

    current_traj_pub = nh.advertise<visualization_msgs::MarkerArray>("optimal_trajectory", 1, true);


    message_filters::Subscriber<sensor_msgs::Image> depth_image_sub_ ;
    depth_image_sub_.subscribe(nh, "camera/depth/image_raw", 5);

    tf::MessageFilter<sensor_msgs::Image> tf_filter_(depth_image_sub_, *listener, "world", 5);
    tf_filter_.registerCallback(depthImageCallback);


    double max_velocity, max_acceleration;
    pnh.param("max_velocity", max_velocity, 2.0);
    pnh.param("max_acceleration", max_acceleration, 5.0);

    Eigen::Vector4d limits(max_velocity, max_acceleration, 0, 0);


    double resolution;
    pnh.param("resolution", resolution, 0.15);
    edrb.reset(new ewok::EuclideanDistanceRingBuffer<POW>(resolution, 1.0));

    double distance_threshold;
    pnh.param("distance_threshold", distance_threshold, 0.5);

    double start_x, start_y, start_z, start_yaw;
    pnh.param("start_x", start_x, 0.0);
    pnh.param("start_y", start_y, 0.0);
    pnh.param("start_z", start_z, 0.0);
    pnh.param("start_yaw", start_yaw, 0.0);

    double middle_x, middle_y, middle_z, middle_yaw;
    pnh.param("middle_x", middle_x, 0.0);
    pnh.param("middle_y", middle_y, 0.0);
    pnh.param("middle_z", middle_z, 0.0);
    pnh.param("middle_yaw", middle_yaw, 0.0);

    double stop_x, stop_y, stop_z, stop_yaw;
    pnh.param("stop_x", stop_x, 0.0);
    pnh.param("stop_y", stop_y, 0.0);
    pnh.param("stop_z", stop_z, 0.0);
    pnh.param("stop_yaw", stop_yaw, 0.0);


    pnh.param("dt", dt, 0.5);
    pnh.param("num_opt_points", num_opt_points, 7);

    ROS_INFO("Started hovering example with parameters: start - %f %f %f %f, middle - %f %f %f %f, stop - %f %f %f %f",
             start_x, start_y, start_z, start_yaw,
             middle_x, middle_y, middle_z, middle_yaw,
             stop_x, stop_y, stop_z, stop_yaw);

    ROS_INFO("dt: %f, num_opt_points: %d", dt, num_opt_points);


    ewok::Polynomial3DOptimization<10> to(limits*0.6);


    {
        typename ewok::Polynomial3DOptimization<10>::Vector3Array path;

        path.push_back(Eigen::Vector3d(start_x, start_y, start_z));
        path.push_back(Eigen::Vector3d(middle_x, middle_y, middle_z));
        path.push_back(Eigen::Vector3d(stop_x, stop_y, stop_z));

        traj = to.computeTrajectory(path);

        visualization_msgs::MarkerArray traj_marker;
        traj->getVisualizationMarkerArray(traj_marker, "gt", Eigen::Vector3d(1,0,1));
        traj_marker_pub.publish(traj_marker);

    }

    spline_optimization.reset(new ewok::UniformBSpline3DOptimization<6>(traj, dt));

    for (int i = 0; i < num_opt_points; i++) {
        spline_optimization->addControlPoint(Eigen::Vector3d(start_x, start_y, start_z));
    }

    spline_optimization->setNumControlPointsOptimized(num_opt_points);
    spline_optimization->setDistanceBuffer(edrb);
    spline_optimization->setDistanceThreshold(distance_threshold);
    spline_optimization->setLimits(limits);

    std_srvs::Empty srv;
    bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    unsigned int i = 0;

    // Trying to unpause Gazebo for 10 seconds.
    while (i <= 10 && !unpaused) {
        ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        unpaused = ros::service::call("/gazebo/unpause_physics", srv);
        ++i;
    }

    if (!unpaused) {
        ROS_FATAL("Could not wake up Gazebo.");
        return -1;
    }
    else {
        ROS_INFO("Unpaused the Gazebo simulation.");
    }

    // Wait for 5 seconds to let the Gazebo GUI show up.
    ros::Duration(5.0).sleep();


    geometry_msgs::Point p;
    p.x = start_x;
    p.y = start_y;
    p.z = start_z;

    //Eigen::Vector3d desired_position(start_x, start_y, start_z);
    double desired_yaw = start_yaw;

    ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f, %f].",
             nh.getNamespace().c_str(),
             start_x,
             start_y,
             start_z, start_yaw);

    trajectory_pub.publish(p);

    ros::Duration(5.0).sleep();

    ros::Timer timer = nh.createTimer(ros::Duration(dt), sendCommandCallback);

    ros::spin();

    f_time.close();
    opt_time.close();
}
