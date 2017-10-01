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
//#define GL_GLEXT_PROTOTYPES 1
//#define GL3_PROTOTYPES 1
//#include <GL/glew.h>

#include "QGLViewer/qglviewer.h"
#include <vector>
#include "boost/thread.hpp"
#include "qevent.h"
#include "lsd_slam_viewer/keyframeMsg.h"
#include "lsd_slam_viewer/keyframeGraphMsg.h"
#include "ros/node_handle.h"
#include "QGLViewer/keyFrameInterpolator.h"

class QApplication;
class KeyFrameGraphDisplay;
class CameraDisplay;
class KeyFrameDisplay;

#include "settings.h"


/**
 * QtWidget used to display the point cloud, using the QGLViewer
 */
class PointCloudViewer : public QGLViewer
{
public:
	PointCloudViewer();
	~PointCloudViewer();
	void reset();
	void addFrameMsg(lsd_slam_viewer::keyframeMsgConstPtr msg);
	void addGraphMsg(lsd_slam_viewer::keyframeGraphMsgConstPtr msg);
	void setNodeHanlder(ros::NodeHandlePtr nh);

protected :
	virtual void draw();
	virtual void init();
	virtual void keyPressEvent(QKeyEvent *e);
	virtual void keyReleaseEvent(QKeyEvent *e);
	virtual QString helpString() const;

private:
	// functions
  void setToVideoSize();

  // variables
  /* displays KeyFrame-Graph */
	KeyFrameGraphDisplay* graphDisplay;

	/* displays only current KeyFrame (which is not yet in the graph). */
	KeyFrameDisplay* currentCamDisplay;

	/* meddle mutex */
	boost::mutex meddleMutex;

	/* reset request from user */
	bool resetRequested;

	// for saving stuff
	std::string save_folder;
	double lastCamTime;
	int lastCamID;
	ros::NodeHandlePtr nh_;

};

//EOF
