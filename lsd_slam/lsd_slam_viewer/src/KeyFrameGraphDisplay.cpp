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

#include "KeyFrameGraphDisplay.h"
#include "KeyFrameDisplay.h"
#include "settings.h"
#include <sstream>
#include <fstream>

#include "ros/package.h"
#include "std_msgs/Float32MultiArray.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32.h>

#include <stdio.h>
#include <math.h>
#include <errno.h>
#include <fenv.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/don.h>

#include <pcl/octree/octree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>



#include <pcl/point_types_conversion.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

#include <cv_bridge/cv_bridge.h>

#include "tf/tf.h"
#include "tf_conversions/tf_eigen.h"

/**
 * Constructor
 */
KeyFrameGraphDisplay::KeyFrameGraphDisplay()
{
  flushPointcloud = false;
  printNumbers = false;

  speed_readings = 0;
  count_frames = 0;
  printed_keyframe = 0;
  last_keyframe_id = 0;
  last_speed = 0.0;
  last_distance = 0.0;
  average_speed = 0.0;
  world_scale = 0.0;

  // total distance traveled
  total_dt = 0.0;
  // total time elapsed
  total_te = 0.0;

//  pcl_viewer_2 = boost::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer 2");

  keyframes.set_capacity(15);

//  pcl_viewer_2->setBackgroundColor(0, 0, 0);
//  pcl_viewer_2->addCoordinateSystem(0.2);
//  pcl_viewer_2->initCameraParameters();


  first_run = true;

  nh_.reset(new ros::NodeHandle("~lsd_point_cloud"));

  image_raw_topic = nh_->resolveName("/lsd_slam/image");
  pub_image_raw = nh_->advertise<sensor_msgs::Image>(image_raw_topic, 1);

  image_mask_topic = nh_->resolveName("/lsd_slam/mask");
  pub_image_mask = nh_->advertise<sensor_msgs::Image>(image_mask_topic, 1);

  image_rgb_mask_topic = nh_->resolveName("/lsd_slam/rgb_mask");
  pub_image_rgb_mask = nh_->advertise<sensor_msgs::Image>(image_rgb_mask_topic, 1);

  image_mask_nf_topic = nh_->resolveName("/lsd_slam/mask_nf");
  pub_image_mask_nf = nh_->advertise<sensor_msgs::Image>(image_mask_nf_topic, 1);

  image_mask_nf2_topic = nh_->resolveName("/lsd_slam/mask_nf2");
  pub_image_mask_nf2 = nh_->advertise<sensor_msgs::Image>(image_mask_nf2_topic, 1);

  est_vel_topic = nh_->resolveName("/lsd_slam/est_vel");
  pub_est_vel = nh_->advertise<std_msgs::Float32>(est_vel_topic, 1);

  plane_slope_topic = nh_->resolveName("/lsd_slam/plane_slope");
  pub_plane_slope = nh_->advertise<std_msgs::Float32>(plane_slope_topic, 1);



  // af-silva TODO
  nh_->param<float>("distance_to_ground", dist_to_ground, 2.7);  // aproximaed value
  nh_->param<std::string>("path_to_log", path_to_file,
      std::string(ros::package::getPath("lsd_slam_viewer") + "/logs/log.csv"));
  nh_->param<float>("distance_to_ground", fps, 50.0);


//  logFile.open(path_to_file.c_str(), std::ios_base::in | std::ios_base::out | std::ios_base::app);
//
//  // If file does not exist, Create new file
//  if (!logFile)
//  {
//    cout << "Cannot open file, file does not exist. Creating new file..";
//    logFile.open(path_to_file.c_str(),
//        std::ios_base::in | std::ios_base::out | std::ios_base::app | std::ios_base::trunc);
//  }
//  // add the header every time execution
//  logFile
//      << "Last KF id; Total Time Elapsed; Instantaneous Speed; Distance 1; Average Speed; Total Distance; World Scale; Distance Mean ; fx \n";
//  logFile.flush();

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Destructor
 */
KeyFrameGraphDisplay::~KeyFrameGraphDisplay()
{
  for (unsigned int i = 0; i < keyframes.size(); i++)
    keyframes[i].reset();

//  pcl_viewer_2->removeAllPointClouds();
//  pcl_viewer_2->close();
//  pcl_viewer_2.reset();
//  plotter->clearPlots();
//  plotter->close();
//  plotter.reset();

  temp_pc.reset();

//  if (logFile.is_open())
//    logFile.close();

}

/**
 *
 */
void KeyFrameGraphDisplay::draw()
{
  dataMutex.lock();

  numRefreshedAlready = 0;

  // Draw Keyframes
  float color[3] = { 0, 0, 1 };

  temp_pc = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA> >();

  Sophus::Sim3f inverse_camToWorld;

  int width = 0;
  int height = 0;
  float fx = 0.0;
  float fy = 0.0;
  float cx = 0.0;
  float cy = 0.0;
  float fxi = 0.0;
  float fyi = 0.0;
  float cxi = 0.0;
  float cyi = 0.0;

  cv::Mat image_rgb_mask;
  cv::Mat image_raw;
  int frame_id = 0;
  first_run = true;

  int current_keyframe_id;
  float ddist;  // int lsd-slam units
  float ddist2;  // int lsd-slam units
  float dtime;  // in seconds

  // just for testing
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr last_pc = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA> >();;

  for (unsigned int i = 0; i < keyframes.size(); i++)
  {

    frame_id = keyframes[i]->id;
    // if true shows cameras poses
//    if (showKFCameras)
//    {
//      keyframes[i]->drawCam(lineTesselation, color);
//    }

    // Draw the last X Keyframes pointclouds
    if ((showKFPointclouds && (int) i > cutFirstNKf) || i > keyframes.size() - 10)
    {
      // draw the last four keyframes point clouds
      keyframes[i]->drawPC(pointTesselation, 1);  // this draws the point cloud using opengl
      current_keyframe_id = keyframes[keyframes.size() - 1]->id;

      if (first_run)
      {
        inverse_camToWorld = keyframes[i]->camToWorld.inverse();
        //focallength = keyframes[i]->getFocalLenght();

        keyframes[i]->image.copyTo(image_rgb_mask);
        keyframes[i]->image.copyTo(image_raw);

        first_run = false;
      }  // end of first run

      for (int j = 0; j < keyframes[i]->pc_pcl->size(); j++)
      {
        pcl::PointXYZRGBA point;
        Sophus::Vector3f pt = (inverse_camToWorld
            * (Sophus::Vector3f((*keyframes[i]->pc_pcl)[j].x, (*keyframes[i]->pc_pcl)[j].y,
                (*keyframes[i]->pc_pcl)[j].z)));
        point.x = pt[0];
        point.y = pt[1];
        point.z = pt[2];
        point.r = (*keyframes[i]->pc_pcl)[j].r;
        point.g = (*keyframes[i]->pc_pcl)[j].g;
        point.b = (*keyframes[i]->pc_pcl)[j].b;
        point.a = (*keyframes[i]->pc_pcl)[j].a;
        temp_pc->push_back(point);

        // justt for testing
//        if(i == keyframes.size() -1)
//          last_pc->push_back(point);

      }

      // af-silva TODO here
      // last keyframe in the buffer
      if (i == (keyframes.size() - 1))
      {
        width = keyframes[i]->width;
        height = keyframes[i]->height;
        fx = keyframes[i]->fx;
        fy = keyframes[i]->fy;
        cx = keyframes[i]->cx;
        cy = keyframes[i]->cy;
        fxi = keyframes[i]->fxi;
        fyi = keyframes[i]->fyi;
        cxi = keyframes[i]->cxi;
        cyi = keyframes[i]->cyi;

        if (last_keyframe_id != current_keyframe_id)
        {
          Sophus::Vector3f pt1 = keyframes[keyframes.size() - 1]->camToWorld * Sophus::Vector3f(1, 1, 1);
          Sophus::Vector3f pt2 = keyframes[keyframes.size() - 2]->camToWorld * Sophus::Vector3f(1, 1, 1);
          ddist = (float)sqrt(pow((double)pt1[0] - (double)pt2[0], 2.0) + pow((double)pt1[1] - (double)pt2[1], 2.0) + pow((double)pt1[2] - (double)pt2[2], 2.0));  // displacement
          world_scale = ((float) keyframes[keyframes.size() - 1]->camToWorld.scale() + (float) keyframes[keyframes.size() - 2]->camToWorld.scale()) / 2.0;
          ddist = ddist * (1.0 / world_scale);
          // the frame_id corresponds to the frame id from the video
          dtime = (keyframes[i]->frame_id / fps) - (keyframes[keyframes.size() - 2]->frame_id / fps);
          last_keyframe_id = keyframes[i]->frame_id;
          total_te = last_keyframe_id / fps;
        }

      }  // last keyframe

    }
  }  // end of for



//  last_pc->width = last_pc->size();
//  last_pc->height = 1;
//  last_pc->is_dense = true;
//  temp_pc->width = last_pc->size();
//  temp_pc->height = 1;
//  temp_pc->is_dense = true;


  // the magic happens here
  if (!temp_pc->empty())
  {

    // ----------------------------------------------------------------------------------------------------------------
    // Filter Point Cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZRGBA> vgf;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sorf;
    // give the original point cloud
    vgf.setInputCloud(temp_pc);
    vgf.setLeafSize(0.02f, 0.02f, 0.02f);  // 5mm (0.05) as default
    vgf.filter(*cloud_filtered);
    sorf.setInputCloud(cloud_filtered);
    sorf.setMeanK(40); // 25
    sorf.setStddevMulThresh(1.0);
    sorf.filter(*cloud_filtered);


    // ----------------------------------------------------------------------------------------------------------------
    // Difference of Normals Based Segmentation
    // see -> http://pointclouds.org/documentation/tutorials/don_segmentation.php

    // see -> http://pointclouds.org/documentation/tutorials/don_segmentation.php
    ///The smallest scale to use in the DoN filter.
    double scale1 = 0.02; // 2cm
    ///The largest scale to use in the DoN filter.
    double scale2 = 0.06; // 6cm

    pcl::search::Search<pcl::PointXYZRGBA>::Ptr tree_;
    tree_.reset(new pcl::search::KdTree<pcl::PointXYZRGBA>(false));
    tree_->setInputCloud(cloud_filtered);
    pcl::NormalEstimationOMP<pcl::PointXYZRGBA, pcl::PointNormal> ne;
    ne.setInputCloud(cloud_filtered);
    ne.setSearchMethod(tree_);
    ne.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max());

    // small radius
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale(new pcl::PointCloud<pcl::PointNormal>);
    ne.setRadiusSearch(scale1);
    ne.compute(*normals_small_scale);

    // large radius
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale(new pcl::PointCloud<pcl::PointNormal>);
    ne.setRadiusSearch(scale2);
    ne.compute(*normals_large_scale);

    pcl::PointCloud<pcl::PointNormal>::Ptr doncloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::DifferenceOfNormalsEstimation<pcl::PointXYZRGBA, pcl::PointNormal, pcl::PointNormal> don;

    pcl::copyPointCloud<pcl::PointXYZRGBA, pcl::PointNormal>(*cloud_filtered, *doncloud);
    don.setInputCloud(cloud_filtered);
    don.setNormalScaleLarge(normals_large_scale);
    don.setNormalScaleSmall(normals_small_scale);

    if (!don.initCompute())
    {
      std::cerr << "Error: Could not intialize DoN feature operator" << std::endl;
      exit(EXIT_FAILURE);
    }
    don.computeFeature(*doncloud);  // Compute DoN

    pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond(new pcl::ConditionOr<pcl::PointNormal>());


//    range_cond->addComparison(
//        pcl::FieldComparison<pcl::PointNormal>::ConstPtr(new pcl::FieldComparison<pcl::PointNormal>(
//            "curvature", pcl::ComparisonOps::LT, 0.25)));

    range_cond->addComparison(
        pcl::FieldComparison<pcl::PointNormal>::ConstPtr(new pcl::FieldComparison<pcl::PointNormal>(
            "normal_y", pcl::ComparisonOps::LT, 0.0)));

    range_cond->addComparison(
        pcl::FieldComparison<pcl::PointNormal>::ConstPtr(new pcl::FieldComparison<pcl::PointNormal>(
            "normal_z", pcl::ComparisonOps::GT, -0.3)));
    range_cond->addComparison(
        pcl::FieldComparison<pcl::PointNormal>::ConstPtr(new pcl::FieldComparison<pcl::PointNormal>(
            "normal_z", pcl::ComparisonOps::LT, 0.3)));

    range_cond->addComparison(
        pcl::FieldComparison<pcl::PointNormal>::ConstPtr(new pcl::FieldComparison<pcl::PointNormal>(
            "normal_x", pcl::ComparisonOps::GT, -0.3)));
    range_cond->addComparison(
        pcl::FieldComparison<pcl::PointNormal>::ConstPtr(new pcl::FieldComparison<pcl::PointNormal>(
            "normal_x", pcl::ComparisonOps::LT, 0.3)));

    // Build the filter
    pcl::ConditionalRemoval<pcl::PointNormal> condrem;
    pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered(new pcl::PointCloud<pcl::PointNormal>);
    condrem.setCondition(range_cond);
    condrem.setInputCloud(doncloud);
    condrem.filter(*doncloud_filtered);  // Apply filter
    doncloud->swap(*doncloud_filtered);  // swaps the original point cloud with the one filtered


    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::Normal>::Ptr normal_out(new pcl::PointCloud<pcl::Normal>);
    for(pcl::PointCloud<pcl::PointNormal>::iterator it = doncloud->begin(); it < doncloud->end(); it++)
    {
      pcl::PointXYZRGBA pt;
      pcl::Normal nl;
      pt.x = it->x;
      pt.y = it->y;
      pt.z = it->z;
      pt.r = 1;
      pt.g = 255;
      pt.b = 1;
      pt.a = 100;
      nl.normal_x = it->normal_x;
      nl.normal_y = it->normal_y;
      nl.normal_z = it->normal_z;
      nl.curvature = it->curvature;
      cloud_out->points.push_back(pt);
      normal_out->points.push_back(nl);
    }
    cloud_out->width = int(cloud_out->points.size());
    cloud_out->height = 1;
    cloud_out->is_dense = true;
    normal_out->width = int(normal_out->points.size());
    normal_out->height = 1;
    normal_out->is_dense = true;


    // ----------------------------------------------------------------------------------------------------------------
    // Segmentation of the ground plane
    // see -> http://docs.pointclouds.org/trunk/group__sample__consensus.html
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_obs(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
    pcl::ExtractIndices<pcl::PointXYZRGBA> ext;
    pcl::SACSegmentationFromNormals<pcl::PointXYZRGBA, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
    //seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setNormalDistanceWeight(0.1);  //0.1
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMinMaxOpeningAngle(0.0873, 3.14); // 0.02 and 3.14 | 0.2  | 0.0873 = 5 degrees to 180 degrees
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.01);  // 0.03  -> 0.003 -> 0.01 -> 0.02 -> 0.01   < 1cm


    seg.setAxis(Eigen::Vector3f(0, 1, 0)); // Y axis
    seg.setEpsAngle(30.0f * (M_PI / 180.0f)); // 30 degrees from the Y axis

    seg.setInputCloud(cloud_out);
    seg.setInputNormals(normal_out);

    // Obtain the plane inliers and coefficients
    seg.segment(*inliers_plane, *coefficients);
    ext.setInputCloud(cloud_out);
    ext.setIndices(inliers_plane);
    ext.setNegative(false);
    ext.filter(*cloud_plane);


    // ----------------------------------------------------------------------------------------------------------------
    // magic here
    cv::Mat image_mask = cv::Mat::zeros(cv::Size(width, height), CV_8U);
    cv::Mat image_mask_dist = cv::Mat::zeros(cv::Size(width, height), CV_8UC3);
    cv::Mat image_mask_nf = cv::Mat::zeros(cv::Size(80, 60), CV_8UC1);
    cv::Mat image_mask_nf2 = cv::Mat::zeros(cv::Size(80, 60), CV_8UC1);


    // TODO af-silva
    float pDistance_mean = 0;
    for (int j = 0; j < cloud_plane->size(); j++)
    {
      float temp_dist = pcl::pointToPlaneDistance(cloud_plane->at(j), coefficients->values[0], coefficients->values[1],
          coefficients->values[2], coefficients->values[3]);
      pDistance_mean += temp_dist;
    }

    pDistance_mean = pDistance_mean / cloud_plane->size();
    //float threshould = pDistance_mean + 0.07; // 0.05<
    float threshould = pDistance_mean + 0.07;

    for (int j = 0; j < cloud_out->size(); j++)
    {
      float distance = pcl::pointToPlaneDistance(cloud_out->at(j), coefficients->values[0], coefficients->values[1],
          coefficients->values[2], coefficients->values[3]);
      float side = coefficients->values[0] * cloud_out->at(j).x + coefficients->values[1] * cloud_out->at(j).y + coefficients->values[2] * cloud_out->at(j).z + coefficients->values[3];

      if (distance > threshould && side > 0)
      {
        cloud_obs->push_back(cloud_out->at(j));

        if (cloud_out->at(j).z != 0.000)
        {
          int x = int((cloud_out->at(j).x / cloud_out->at(j).z) / fxi - cxi) + cx;
          int y = int((cloud_out->at(j).y / cloud_out->at(j).z) / fyi - cyi) + cy;

          if ((x > 0 && x < width) && (y > 0 && y < height))
          {
            image_mask.at<uchar>(cv::Point2i((int) x, (int) y)) = 255;
            if (!image_rgb_mask.empty())
              image_rgb_mask.at<cv::Vec3b>(cv::Point((int) x, (int) y)) = cv::Vec3b(0, 0, 255);
          }
        }
      }
    }

    float slope_beta = asin(coefficients->values[2]);
//    float alfa = - acos(coefficients->values[1]/cos(beta));


    // Publish an estimation of the speed
    // ------------------------------------------------------------------------------------------------------------- //

    // TODO af-silva
    float distance = 0.0;
    float speed = 0.0;
    float average_speed = 0.0;
    float dstep = 0.0;

    // Distance from the plane to the camera pose
    float cam_to_plane;
    cam_to_plane = coefficients->values[3];  // this is de "d" from the coefficients
    if (printed_keyframe != last_keyframe_id)
    {
      // the camera cannot be below the ground, and the time cannot jump back (it will be nice but impossible)
      // the distanc can be negative but we assume that the camera goes always forward
      if (ddist > 0.000001 && dtime > 0.000001 && cam_to_plane > 0.0001)
      {
        distance = (ddist * dist_to_ground) / cam_to_plane;
        total_dt += distance;
        speed = distance / dtime;  // between two points
        average_speed = total_dt / total_te;

        if (last_speed != speed)
          last_speed = speed;
        else
          speed = 0.0;
      }
      count_frames = 0;
    }
    else if (count_frames > 3)  // filter false positives
    {
      last_speed = 0;
    }






    // ------------------------------------------------------------------------------------------------------------- //

    cv::resize(image_mask, image_mask_nf, image_mask_nf.size(), 0, 0, CV_INTER_AREA);
    cv::threshold(image_mask_nf, image_mask_nf2, 1, 255, CV_THRESH_BINARY_INV);


    std_msgs::Float32 msg_vel;
    msg_vel.data = last_speed;
    pub_est_vel.publish(msg_vel);

    std_msgs::Float32 msg_slope;
    msg_slope.data = slope_beta;
    pub_plane_slope.publish(msg_slope);


    if (!image_rgb_mask.empty())
    {
      sensor_msgs::ImagePtr iMsg;
      iMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_rgb_mask).toImageMsg();
      pub_image_rgb_mask.publish(iMsg);
    }

    if (!image_mask.empty())
    {
      sensor_msgs::ImagePtr iMsg;
      iMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_mask).toImageMsg();
      pub_image_mask.publish(iMsg);
    }

    if (!image_mask_nf.empty())
    {
      sensor_msgs::ImagePtr iMsg;
      iMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_mask_nf).toImageMsg();
      pub_image_mask_nf.publish(iMsg);
    }

    if (!image_mask_nf2.empty())
    {
      sensor_msgs::ImagePtr iMsg;
      iMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_mask_nf2).toImageMsg();
      pub_image_mask_nf2.publish(iMsg);
    }

    usleep(100); // TODO danger

    if (!image_raw.empty())
    {
      sensor_msgs::ImagePtr iMsg;
      std_msgs::Header header;
      header.frame_id = boost::lexical_cast<std::string>(frame_id);
      iMsg = cv_bridge::CvImage(header, "bgr8", image_raw).toImageMsg();
      pub_image_raw.publish(iMsg);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Viewer


//    pcl_viewer_2->removeAllPointClouds();

//    // test only
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> white(last_pc, 255, 255, 255);
//    if (!pcl_viewer_2->updatePointCloud(last_pc, white, ("last_pc")))
//    {
//      pcl_viewer_2->addPointCloud<pcl::PointXYZRGBA>(last_pc, white, ("last_pc"));
//      pcl_viewer_2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, ("last_pc"));
//    }

//    // test only
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> white(temp_pc, 255, 255, 255);
//    if (!pcl_viewer_2->updatePointCloud(temp_pc, white, ("temp_pc")))
//    {
//      pcl_viewer_2->addPointCloud<pcl::PointXYZRGBA>(temp_pc, white, ("temp_pc"));
//      pcl_viewer_2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, ("temp_pc"));
//    }





//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> red(cloud_obs, 255, 0, 0);
//    if (!pcl_viewer_2->updatePointCloud(cloud_obs, red, ("cloud_out")))
//    {
//      pcl_viewer_2->addPointCloud<pcl::PointXYZRGBA>(cloud_obs, red, ("cloud_out"));
//      pcl_viewer_2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, ("cloud_out"));
//    }
//
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> green(cloud_plane, 0, 255, 0);
//    if (!pcl_viewer_2->updatePointCloud(cloud_plane, green, ("cloud")))
//    {
//      pcl_viewer_2->addPointCloud<pcl::PointXYZRGBA>(cloud_plane, green, ("cloud"));
//      pcl_viewer_2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, ("cloud"));
//
//    }



    // just for testing
//    pcl::PointCloud<pcl::Normal>::Ptr plane_normal (new pcl::PointCloud<pcl::Normal>);
//    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> plane_ne;
//    plane_ne.setInputCloud (cloud_plane);
//    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr plane_ree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
//    plane_ne.setSearchMethod (plane_ree);
//    plane_ne.setRadiusSearch (0.03);
//    plane_ne.compute(*plane_normal);
//
//    pcl_viewer_2->addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal>(cloud_plane, plane_normal, 10, 0.03, "normals");


  }

  //pcl_viewer_2->spinOnce(100);


//  if (flushPointcloud)  // saves the global point cloud to file
//  {
//    printf("Flushing Pointcloud to %s!\n", (ros::package::getPath("lsd_slam_viewer") + "/pc_tmp.ply").c_str());
//    std::ofstream f((ros::package::getPath("lsd_slam_viewer") + "/pc_tmp.ply").c_str());
//    int numpts = 0;
//
//    for (unsigned int i = 0; i < keyframes.size(); i++)
//    {
//      if ((int) i > cutFirstNKf)
//        numpts += keyframes[i]->flushPC(&f);
//    }
//
//    f.flush();
//    f.close();
//
//    std::ofstream f2((ros::package::getPath("lsd_slam_viewer") + "/pc.ply").c_str());
//    f2 << std::string("ply\n");
//    f2 << std::string("format binary_little_endian 1.0\n");
//    f2 << std::string("element vertex ") << numpts << std::string("\n");
//    f2 << std::string("property float x\n");
//    f2 << std::string("property float y\n");
//    f2 << std::string("property float z\n");
//    f2 << std::string("property float intensity\n");
//    f2 << std::string("end_header\n");
//
//    std::ifstream f3((ros::package::getPath("lsd_slam_viewer") + "/pc_tmp.ply").c_str());
//    while (!f3.eof())
//      f2.put(f3.get());
//
//    f2.close();
//    f3.close();
//    int sys_res;
//    sys_res = system(("rm " + ros::package::getPath("lsd_slam_viewer") + "/pc_tmp.ply").c_str());
//    flushPointcloud = false;
//    printf("Done Flushing Pointcloud with %d points!\n", numpts);
//
//  }  //flushPointcloud
//
//  if (printNumbers)  // print statistics to the console
//  {
//    int totalPoint = 0;
//    int visPoints = 0;
//
//    for (unsigned int i = 0; i < keyframes.size(); i++)
//    {
//      totalPoint += keyframes[i]->totalPoints;
//      visPoints += keyframes[i]->displayedPoints;
//    }
//
//    printf("Have %d points, %d keyframes, %d constraints. Displaying %d points.\n",
//        totalPoint, (int) keyframes.size(), (int) constraints.size(), visPoints);
//    printNumbers = false;
//  }

//  if (showConstraints)  // lines between camera poses
//  {
//    // draw constraints
//    glLineWidth(lineTesselation);
//    glBegin(GL_LINES);
//    for (unsigned int i = 0; i < constraints.size(); i++)
//    {
//      if (constraints[i].from == 0 || constraints[i].to == 0)
//        continue;
//
//      double colorScalar = std::max(0.0, std::min(1.0, constraints[i].err / 0.05));
//      glColor3f(colorScalar, 1 - colorScalar, 0);
//
//      Sophus::Vector3f t = constraints[i].from->camToWorld.translation();
//      glVertex3f((GLfloat) t[0], (GLfloat) t[1], (GLfloat) t[2]);
//
//      t = constraints[i].to->camToWorld.translation();
//      glVertex3f((GLfloat) t[0], (GLfloat) t[1], (GLfloat) t[2]);
//
//    }
//    glEnd();
//  }  // showConstraints

  dataMutex.unlock();
  count_frames++;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Add new KeyFrame point cloud to the internal memory
 */
void KeyFrameGraphDisplay::addMsg(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
  dataMutex.lock();

  if (keyframesByID.count(msg->id) == 0)
  {
    boost::shared_ptr<KeyFrameDisplay> disp = boost::make_shared<KeyFrameDisplay>();
    keyframesByID[msg->id] = disp;
    keyframes.push_back(disp);
    //printf("\nAdded new KF, now there are %d!\n", (int) keyframes.size());
  }

  keyframesByID[msg->id]->setFrom(msg);

  dataMutex.unlock();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 *
 */
void KeyFrameGraphDisplay::addGraphMsg(lsd_slam_viewer::keyframeGraphMsgConstPtr msg)
{
  dataMutex.lock();

  constraints.resize(msg->numConstraints);

  assert(msg->constraintsData.size() == sizeof(GraphConstraint) * msg->numConstraints);

  GraphConstraint* constraintsIn = (GraphConstraint*) msg->constraintsData.data();
  for (int i = 0; i < msg->numConstraints; i++)
  {
    constraints[i].err = constraintsIn[i].err;
    constraints[i].from = 0;
    constraints[i].to = 0;

    if (keyframesByID.count(constraintsIn[i].from) != 0)
      constraints[i].from = keyframesByID[constraintsIn[i].from];
//		else
//			printf("ERROR: graph update contains constraints for %d -> %d, but I dont have a frame %d!\n",
//					constraintsIn[i].from,
//					constraintsIn[i].to,
//					constraintsIn[i].from);

    if (keyframesByID.count(constraintsIn[i].to) != 0)
      constraints[i].to = keyframesByID[constraintsIn[i].to];
//		else
//			printf("ERROR: graph update contains constraints for %d -> %d, but I dont have a frame %d!\n",
//					constraintsIn[i].from,
//					constraintsIn[i].to,
//					constraintsIn[i].to);
  }

  GraphFramePose* graphPoses = (GraphFramePose*) msg->frameData.data();
  int numGraphPoses = msg->numFrames;

  assert(msg->frameData.size() == sizeof(GraphFramePose) * msg->numFrames);

  for (int i = 0; i < numGraphPoses; i++)
  {
    if (keyframesByID.count(graphPoses[i].id) == 0)
    {
      //printf("ERROR: graph update contains pose for frame %d, but I dont have a frame %d!\n", graphPoses[i].id, graphPoses[i].id);
    }
    else
      memcpy(keyframesByID[graphPoses[i].id]->camToWorld.data(), graphPoses[i].camToWorld, 7 * sizeof(float));
  }

  dataMutex.unlock();
//	printf("graph update: %d constraints, %d poses\n", msg->numConstraints, msg->numFrames);

}

//EOF

