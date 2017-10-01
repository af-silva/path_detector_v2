/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam> 
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once


#include <ros/ros.h>
#include "IOWrapper/Output3DWrapper.h"

//#define BOOST_CB_DISABLE_DEBUG // The Debug Support has to be disabled, otherwise the code produces a runtime error.
#include <boost/circular_buffer.hpp>


#include "lsd_slam_viewer/keyframeMsg.h"

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include "pcl_ros/point_cloud.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "image_transport/image_transport.h"
#include <cv_bridge/cv_bridge.h>

namespace lsd_slam
{


class Frame;
class KeyFrameGraph;

struct InputPointDense
{
	float idepth;
	float idepth_var;
	unsigned char color[4];
};

struct GraphConstraint
{
	int from;
	int to;
	float err;
};

struct GraphFramePose
{
	int id;
	float camToWorld[7];
};




struct KFPointCloud
{
    int id;
    double time;
    bool isKeyFrame;
    int width;
    int height;
    int totalPoints, displayedPoints;
    float fx,fy,cx,cy;
    Sophus::Sim3f  camToWorld;
    InputPointDense* originalInput;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud;


    KFPointCloud(void)
    {
      id = 0;
      time = 0;
      isKeyFrame = false;
      width=0;
      height = 0;
      totalPoints = 0;
      displayedPoints = 0;
      fx= 0.0;
      fy= 0.0;
      cx= 0.0;
      cy= 0.0;
      originalInput = 0;
      pointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >();
    }

};


/** Addition to LiveSLAMWrapper for ROS interoperability. */
class ROSOutput3DWrapper : public Output3DWrapper
{
public:

	// initializes cam-calib independent stuff
	ROSOutput3DWrapper(int width, int height);
	~ROSOutput3DWrapper();

	virtual void publishKeyframeGraph(KeyFrameGraph* graph);

	// publishes a keyframe. if that frame already existis, it is overwritten, otherwise it is added.
	virtual void publishKeyframe(Frame* f);

	// published a tracked frame that did not become a keyframe (i.e. has no depth data)
	virtual void publishTrackedFrame(Frame* f);

	// publishes graph and all constraints, as well as updated KF poses.
	virtual void publishTrajectory(std::vector<Eigen::Matrix<float, 3, 1>> trajectory, std::string identifier);

	virtual void publishTrajectoryIncrement(Eigen::Matrix<float, 3, 1> pt, std::string identifier);

	virtual void publishDebugInfo(Eigen::Matrix<float, 20, 1> data);

	virtual void pulbishDepthMap(cv::Mat& depth_map);

	// publish the current point cloud without any transformation

	virtual void publishPointCloud(KFPointCloud* kf);
	int publishLvl;

	ros::Timer timer_pub_cloud; // check emergency stop status
	
	void pubblishPointCloudTimer(const ros::TimerEvent&);

private:
	int width, height;

	std::string liveframe_channel;
	ros::Publisher liveframe_publisher;

	std::string keyframe_channel;
	ros::Publisher keyframe_publisher;

	std::string graph_channel;
	ros::Publisher graph_publisher;

	std::string debugInfo_channel;
	ros::Publisher debugInfo_publisher;

	std::string pose_channel;
	ros::Publisher pose_publisher;

	std::string depthmap_channel;
	ros::Publisher depthmap_publisher;

	std::string image_channel;
	ros::Publisher image_publisher;

	std::string pc_path_channel;
	ros::Publisher pc_path_publisher;

	// TODO - implement
	std::string pointcloud2_channel;
	ros::Publisher pointcloud2_publisher;

	// ROS Node handler
	ros::NodeHandle nh_;

	// other much needed variables
	pcl::PointXYZRGB point_invalid_;
	pcl::PointXYZRGBA pointRGBA_invalid_;

	cv::Mat imageToPathDetectorRGB;

	// this vector holds all the point clouds from each KeyFrame

	//std::vector<KeyFramePointCloud*> keyframes; // can hold an undefined quantity of KeyFrames
	boost::circular_buffer<KFPointCloud*> keyframes; // only holds a predefined quantity of KeyFrames
	std::map<int, KFPointCloud*> keyframesByID;


};
}
