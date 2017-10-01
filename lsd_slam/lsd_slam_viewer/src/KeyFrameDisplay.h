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
#pragma once

#undef Success
#include <Eigen/Core>
#include "QGLViewer/qglviewer.h"
#include "lsd_slam_viewer/keyframeMsg.h"
#include "sophus/sim3.hpp"
#include <sstream>
#include <fstream>


//TODO - REMOVE
#include <pcl/console/parse.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>


/* ?? */
struct MyVertex
{
	float point[3];
	uchar color[4];
};


/* ?? */
struct InputPointDense
{
	float idepth;
	float idepth_var;
	uchar color[4];
};


/**
 * Stores a Point Cloud associated to a Keyframe from the LSD-SLAM algorithm
 */
class KeyFrameDisplay
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    KeyFrameDisplay();
    ~KeyFrameDisplay();

    void setFrom(lsd_slam_viewer::keyframeMsgConstPtr msg);
    void drawCam(float lineWidth = 1, float* color = 0);
    void drawPC(float pointSize = 1, float alpha = 1);
    void refreshPC();
    int flushPC(std::ofstream* f);
    float getFocalLenght();

    int id;
    int frame_id;
    double time;

    int totalPoints, displayedPoints;

    /* camera pose, may be updated by KeyFrame-graph. */
    Sophus::Sim3f camToWorld;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc_pcl;
    cv::Mat bin_image;
    cv::Mat image;

    int width, height;

    /* camera parameter, fixed. */
    float fx,fy,cx,cy;
    float fxi,fyi,cxi,cyi;



  private:




    float my_scaledTH, my_absTH, my_scale;
    int my_minNearSupport;
    int my_sparsifyFactor;

    /* pointcloud data & respective buffer */
    InputPointDense* originalInput;



    /* buffer & how many */
    GLuint vertexBufferId;
    int vertexBufferNumPoints;

    /* true if the vertixBufferID is valid (doesnt mean the data in there is still valid) */
    bool vertexBufferIdValid;

    /* true if the vertexBufferID contains valid data */
    bool glBuffersValid;



};

//EOF
