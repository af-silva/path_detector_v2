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
 *
 */

#ifndef KEYFRAMEGRAPHDISPLAY_H_
#define KEYFRAMEGRAPHDISPLAY_H_

#include "lsd_slam_viewer/keyframeGraphMsg.h"
#include "lsd_slam_viewer/keyframeMsg.h"
#include "boost/thread.hpp"
#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/lexical_cast.hpp>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/vtk.h>
#include <pcl/visualization/window.h>
#include <pcl/visualization/pcl_plotter.h>

#include <cv.h>
#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>

class KeyFrameDisplay;

/* Constraints between camera poses (the lines between the cameras) */
struct GraphConstraint
{
  int from;
  int to;
  float err;
};

/* Constraints between camera poses (the lines between the cameras) as pointer */
struct GraphConstraintPt
{
  boost::shared_ptr<KeyFrameDisplay> from;
  boost::shared_ptr<KeyFrameDisplay> to;
  float err;
};

/* */
struct GraphFramePose
{
  int id;
  float camToWorld[7];
};

/* Class that holds the global point cloud information */
class KeyFrameGraphDisplay
{
public:
  // function
  KeyFrameGraphDisplay();
  virtual ~KeyFrameGraphDisplay();
  void draw();
  void addMsg(lsd_slam_viewer::keyframeMsgConstPtr msg);
  void addGraphMsg(lsd_slam_viewer::keyframeGraphMsgConstPtr msg);

  // variables
  bool flushPointcloud;
  bool printNumbers;

private:

  std::map<int, boost::shared_ptr<KeyFrameDisplay> > keyframesByID;
  // this vector holds all the point clouds from each KeyFrame
  boost::circular_buffer<boost::shared_ptr<KeyFrameDisplay> > keyframes;
  std::vector<GraphConstraintPt> constraints;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer_2;
  boost::shared_ptr<pcl::visualization::PCLPlotter> plotter;

  boost::shared_ptr<std::vector<std::pair<double, double> > > v_pitch;
  boost::shared_ptr<std::vector<std::pair<double, double> > > v_roll;
  boost::shared_ptr<std::vector<std::pair<double, double> > > v_height;


  int last_keyframe_id;
  int printed_keyframe;
  int count_frames;
  int speed_readings;
  float average_speed;
  float last_speed;
  float last_distance;
  float total_dt;
  float total_te;
  float fps;
  float dist_to_ground;
  float world_scale;

  std::string path_to_file;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp_pc;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr last_cloud_filtred;

  boost::mutex dataMutex;
  bool first_run;
  ros::NodeHandlePtr nh_;

  std::string pub_pointcloudTopic;
  std::string pub_camToGroudTopic;

  ros::Publisher pub_pointcloud;
  ros::Publisher pub_camToGround;

  std::string image_raw_topic;
  std::string image_mask_topic;
  std::string image_rgb_mask_topic;
  std::string image_mask_dist_topic;
  std::string image_mask_nf_topic;
  std::string image_mask_nf2_topic;
  std::string est_vel_topic;
  std::string plane_slope_topic;

  ros::Publisher pub_image_raw;
  ros::Publisher pub_image_mask;
  ros::Publisher pub_image_rgb_mask;
  ros::Publisher pub_image_mask_dist;
  ros::Publisher pub_image_mask_nf;
  ros::Publisher pub_image_mask_nf2;

  ros::Publisher pub_est_vel;
  ros::Publisher pub_plane_slope;

  std::string keyframe_channel;
  ros::Subscriber sub_keyframe;

  std::ofstream logFile;



};

#endif /* KEYFRAMEGRAPHDISPLAY_H_ */
