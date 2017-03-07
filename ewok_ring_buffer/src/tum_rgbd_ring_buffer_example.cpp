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

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sophus/se3.hpp>

#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

#include <fstream>
#include <chrono>



class DatasetReader {
public:
	DatasetReader(const std::string & path) : path(path+ "/") {
		intrinsics = Eigen::Vector4f(517.3, 516.5, 318.6, 255.3);
		std::string fusion_path = path + "/depth_gt.txt";
		std::ifstream f(fusion_path.c_str());

        std::cout << "Opening file: " << fusion_path << std::endl;

		std::string timestamp;
		float rgb_timestamp;
		float depth_timestamp;
		while (!f.eof()) {
			std::string temp;
			getline(f, temp);
			std::stringstream stream(temp);
			Eigen::Vector3f translation;
			Eigen::Quaternionf quaternion;
			std::string depth_name, rgb_name;

            stream >> depth_timestamp;
            stream >> depth_name;

            stream >> timestamp;
			stream >> translation[0];
			stream >> translation[1];
			stream >> translation[2];
			stream >> quaternion.coeffs()[0];
			stream >> quaternion.coeffs()[1];
			stream >> quaternion.coeffs()[2];
			stream >> quaternion.coeffs()[3];

			timestamps.push_back(timestamp);
			depth_image_files.push_back(depth_name);
			poses.push_back(Sophus::SE3f(quaternion, translation));
		}
		std::cout << "Loaded " << depth_image_files.size() << " images" << std::endl;
	}

	cv::Mat get_d(int i) {
			return cv::imread(path + depth_image_files[i],
					CV_LOAD_IMAGE_UNCHANGED);
	}
	Eigen::Matrix4f get_pose(int i) {
		return poses[i].matrix();
	}
	std::string get_timestamp(int i) {
		return timestamps[i];
	}
	size_t num_images() {
		return depth_image_files.size()-1;
	}
private:
	std::string path;
	std::vector<std::string> depth_image_files;
	std::vector<std::string> timestamps;
	std::vector<Sophus::SE3f> poses;
	Eigen::Vector4f intrinsics;
};



const static int POW = 6;

ros::Publisher occ_marker_pub, free_marker_pub, dist_marker_pub, octomap_pub;

std::ofstream f_time;

bool initialized = false;

const double resolution = 0.1;

//ewok::QuasiEuclideanDistanceRingBuffer<POW> rrb(0.1, 1.0);
ewok::EuclideanDistanceRingBuffer<POW> rrb(resolution, 1.0);

octomap::OcTree tree(resolution);



void processImage(const cv::Mat & img, const Eigen::Matrix4f & T_w_c)
{
    const float fx = 554.254691191187;
    const float fy = 554.254691191187;
    const float cx = 320.5;
    const float cy = 240.5;

    uint16_t * data = (uint16_t *) img.data;

    ewok::EuclideanDistanceRingBuffer<POW>::PointCloud cloud1;
    octomap::Pointcloud octomap_cloud;

    const int subsample = 4;

    for(int u=0; u < img.cols; u += subsample) {
        for(int v=0; v < img.rows; v += subsample) {
            uint16_t uval = data[v*img.cols + u];

            //ROS_INFO_STREAM(val);

            if(uval > 0) {
                float val = uval/5000.0;
                Eigen::Vector4f p;
                p(0) = val*(u - cx)/fx;
                p(1) = val*(v - cy)/fy;
                p(2) = val;
                p(3) = 1;

                p = T_w_c * p;

                //ROS_INFO_STREAM(p);
                cloud1.push_back(p);
                octomap_cloud.push_back(p(0), p(1), p(2));
            }
        }
    }

    Eigen::Vector3f origin = (T_w_c * Eigen::Vector4f(0,0,0,1)).head<3>();

    if(!initialized) {
        Eigen::Vector3i idx;
        rrb.getIdx(origin, idx);

        ROS_INFO_STREAM("Origin: " << origin.transpose() << " idx " << idx.transpose());

        rrb.setOffset(idx);

        initialized = true;
    } else {
        Eigen::Vector3i origin_idx, offset, diff;
        rrb.getIdx(origin, origin_idx);

        offset = rrb.getVolumeCenter();

        diff = origin_idx - offset;

        if(diff.array().any()) {
            //ROS_INFO("Moving Volume");
            rrb.moveVolume(diff.head<3>());
        }


    }


    octomap::point3d sensor_origin(origin(0), origin(1), origin(2));

    auto t1 = std::chrono::high_resolution_clock::now();

    rrb.insertPointCloud(cloud1, origin);

    auto t2 = std::chrono::high_resolution_clock::now();

    rrb.updateDistance();

    auto t3 = std::chrono::high_resolution_clock::now();

    tree.insertPointCloud(octomap_cloud, sensor_origin);

    auto t4 = std::chrono::high_resolution_clock::now();

    f_time << std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count()
           << " " << std::chrono::duration_cast<std::chrono::nanoseconds>(t3-t2).count()
           << " " << std::chrono::duration_cast<std::chrono::nanoseconds>(t4-t3).count() << "\n" ;


    visualization_msgs::Marker m_occ, m_free, m_dist;
    rrb.getMarkerOccupied(m_occ);
    rrb.getMarkerFree(m_free);
    rrb.getMarkerDistance(m_dist, 0.5);


    occ_marker_pub.publish(m_occ);
    free_marker_pub.publish(m_free);
    dist_marker_pub.publish(m_dist);

    {
      octomap_msgs::Octomap msg;
      octomap_msgs::binaryMapToMsg(tree, msg);
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "world";
      octomap_pub.publish(msg);
      std::cout << std::endl;
    }


}

int main(int argc, char** argv) {

    ros::init(argc, argv, "tum_rgbd_rolling_buffer_test");
    ros::NodeHandle nh;

    DatasetReader dr(argv[1]);

    f_time.open(argv[2]);


    occ_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/occupied", 5, true);
    free_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/free", 5, true);
    dist_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/distance", 5, true);
    octomap_pub = nh.advertise<octomap_msgs::Octomap>("octomap", 5, true);

    ros::Duration(1.0).sleep();

    for(int i=0; i<dr.num_images(); i++) {
        //std::cout << "Processing image: " << i << std::endl;
        processImage(dr.get_d(i), dr.get_pose(i));
        ros::spinOnce();
    }

    f_time.close();

    //ros::spin();

    return 0;
}