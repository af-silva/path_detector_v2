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
* along with dvo. If not, see <http://www.gnu.org/licenses/>.
*/


#include "ros/ros.h"
#include "ros/node_handle.h"
#include "boost/shared_ptr.hpp"
#include "boost/thread.hpp"
#include "settings.h"
#include "PointCloudViewer.h"

#include <dynamic_reconfigure/server.h>
#include "lsd_slam_viewer/LSDSLAMViewerParamsConfig.h"
#include <qapplication.h>


#include "lsd_slam_viewer/keyframeGraphMsg.h"
#include "lsd_slam_viewer/keyframeMsg.h"



#include "boost/foreach.hpp"
#include "rosbag/bag.h"
#include "rosbag/query.h"
#include "rosbag/view.h"


PointCloudViewer* viewer = 0;


/**
 * Dynamic reconfigure
 */
void dynConfCb(lsd_slam_viewer::LSDSLAMViewerParamsConfig &config, uint32_t level)
{

	pointTesselation = config.pointTesselation;
	lineTesselation = config.lineTesselation;

	keepInMemory = config.keepInMemory;
	showKFCameras = config.showKFCameras;
//	showKFPointclouds = config.showKFPointclouds;
	showKFPointclouds = false;
	showConstraints = config.showConstraints;
	showCurrentCamera = config.showCurrentCamera;
	showCurrentPointcloud = config.showCurrentPointcloud;


	scaledDepthVarTH = exp10( config.scaledDepthVarTH );
	absDepthVarTH = exp10( config.absDepthVarTH );
	minNearSupport = config.minNearSupport;
	sparsifyFactor = config.sparsifyFactor;
	cutFirstNKf = config.cutFirstNKf;




}


///////////////////////////////////////////////////////////////////////////////////////////////////


/**
 *\brief Callback for messages of type keyframeMsg that contain the following data:
 *
 * Header:
 *    int32 id
 *    float64 time
 *    bool isKeyframe
 *
 * Camera parameter (fx fy cx cy), width, height:
 *    float32 fx - focal lenght in x
 *    float32 fy - focal lenfht in y
 *    float32 cx - image center in x
 *    float32 cy - image center in y
 *    uint32 height
 *    uint32 width
 *
 * Data as InputPointDense (float idepth, float idepth_var, uchar color[4]), width x height
 * may be empty, in that case no associated pointcloud is ever shown:
 *    uint8[] pointcloud
 */
void frameCb(lsd_slam_viewer::keyframeMsgConstPtr msg)
{

	if(msg->time > lastFrameTime) return;

	if(viewer != 0)
	{
		viewer->addFrameMsg(msg);

	}

}


///////////////////////////////////////////////////////////////////////////////////////////////////


/**
 * \brief Callback for messages of type keyframeGraphMsg that contain the following data:
 *
 * Data as serialization of sim(3)'s: (int id, float[7] camToWorld)
 *    uint32 numFrames
 *    uint8[] frameData
 *
 * Constraints (int from, int to, float err)
 *    uint32 numConstraints
 *    uint8[] constraintsData
 *
 */
void graphCb(lsd_slam_viewer::keyframeGraphMsgConstPtr msg)
{
	if(viewer != 0)
		viewer->addGraphMsg(msg);
}


///////////////////////////////////////////////////////////////////////////////////////////////////


/**
 * \brief Main ROS thread lopp responsible for executing the callbacks.
 * While the node is running the ros::spin is executed, calliing all the registered callbacks.
 */
void rosThreadLoop( int argc, char** argv )
{
	printf("Started ROS thread\n");


	dynamic_reconfigure::Server<lsd_slam_viewer::LSDSLAMViewerParamsConfig> srv;
	srv.setCallback(dynConfCb);


	ros::NodeHandlePtr nh;
	nh.reset(new ros::NodeHandle());


	ros::Subscriber liveFrames_sub = nh->subscribe(nh->resolveName("lsd_slam/liveframes"),1, frameCb);
	ros::Subscriber keyFrames_sub  = nh->subscribe(nh->resolveName("lsd_slam/keyframes"),20, frameCb);
	ros::Subscriber graph_sub      = nh->subscribe(nh->resolveName("lsd_slam/graph"),10, graphCb);


	viewer->setNodeHanlder(nh);

	ros::Rate rate = ros::Rate(30);
	while(ros::ok()){

	  ros::spinOnce();

	  rate.sleep();
	}

	ros::shutdown();

	printf("Exiting ROS thread\n");


	exit(1);
}



///////////////////////////////////////////////////////////////////////////////////////////////////


/**
 * \brief Main function that starts everything
 */
int main( int argc, char** argv )
{


	printf("Started QApplication thread [new]\n");
	// Read command lines arguments.
	QApplication application(argc,argv);

	ros::init(argc, argv, "viewer");
	ROS_INFO("lsd_slam_viewer started");

	// Instantiate the viewer.
	viewer = new PointCloudViewer();

  viewer->setWindowTitle("PointCloud Viewer Stand-alone");

	// Make the viewer window visible on screen.
	viewer->show();

	boost::thread rosThread;

		// start ROS thread
  rosThread = boost::thread(rosThreadLoop, argc, argv); // <--- this one

	application.exec();

	printf("Shutting down... \n");
	ros::shutdown();
	rosThread.join();
	delete(viewer);
	printf("Done. \n");

}

//EOF
