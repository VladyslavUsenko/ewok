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

#include <ros/ros.h>
#include <ewok/ed_ring_buffer.h>


static const int POW = 6;
static const int N = (1 << POW);

int main(int argc, char** argv) {

    ros::init(argc, argv, "rolling_buffer_test");
    ros::NodeHandle nh;

    ros::Publisher occ_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/occupied", 1, true);
    ros::Publisher free_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/free", 1, true);
    ros::Publisher updated_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/updated", 1, true);
    ros::Publisher dist_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/distance", 1, true);

    //ewok::QuasiEuclideanDistanceRingBuffer<POW> rrb(0.1, 1.0);
    //ewok::QuasiEuclideanDistanceRingBuffer<POW>::PointCloud cloud;

    ewok::EuclideanDistanceRingBuffer<POW> rrb(0.1, 1.0);
    ewok::EuclideanDistanceRingBuffer<POW>::PointCloud cloud;

    cloud.push_back(Eigen::Vector4f(-1, 1, 2, 0));
    cloud.push_back(Eigen::Vector4f(1, 1, 2, 0));
    cloud.push_back(Eigen::Vector4f(-1, -1, 2, 0));
    cloud.push_back(Eigen::Vector4f(1, -1, 2, 0));
    cloud.push_back(Eigen::Vector4f(0, 0, 5, 0));

    cloud.push_back(Eigen::Vector4f(0, 3, 5, 0));
    cloud.push_back(Eigen::Vector4f(3, 0, 5, 0));
    cloud.push_back(Eigen::Vector4f(0, 3, -5, 0));
    cloud.push_back(Eigen::Vector4f(3, 0, -5, 0));


    rrb.insertPointCloud(cloud, Eigen::Vector3f(0,0,0));
    rrb.updateDistance();

    rrb.insertPointCloud(cloud, Eigen::Vector3f(0,0,0));
    rrb.updateDistance();

    rrb.insertPointCloud(cloud, Eigen::Vector3f(0,0,0));
    rrb.updateDistance();

    ros::Rate r(1);
    while (ros::ok())
    {
        r.sleep();

        visualization_msgs::Marker m_occ, m_free, m_dist, m_updated;
        rrb.getMarkerOccupied(m_occ);
        rrb.getMarkerFree(m_free);
        rrb.getMarkerUpdated(m_updated);
        rrb.getMarkerDistance(m_dist, 0.9);

        occ_marker_pub.publish(m_occ);
        free_marker_pub.publish(m_free);
        updated_marker_pub.publish(m_updated);
        dist_marker_pub.publish(m_dist);

        rrb.moveVolume(Eigen::Vector3i(1,0,0));

        ros::spinOnce();
    }

    return 0;
}