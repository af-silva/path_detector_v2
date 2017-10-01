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

#define GL_GLEXT_PROTOTYPES 1
#include "PointCloudViewer.h"
#include "qfiledialog.h"
#include "qcoreapplication.h"
#include <stdio.h>
#include "settings.h"
#include "ros/package.h"

#include <zlib.h>
#include <iostream>


#include <GL/glx.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include "QGLViewer/manipulatedCameraFrame.h"

#include "KeyFrameDisplay.h"
#include "KeyFrameGraphDisplay.h"

#include <iostream>
#include <fstream>


/**
 * Constructor of the Class. Extends the QGLViewer class that extends QGLWidget based on the basic QT QWidget
 */
PointCloudViewer::PointCloudViewer()
{

	setPathKey(Qt::Key_0,0);
	setPathKey(Qt::Key_1,1);
	setPathKey(Qt::Key_2,2);
	setPathKey(Qt::Key_3,3);
	setPathKey(Qt::Key_4,4);
	setPathKey(Qt::Key_5,5);
	setPathKey(Qt::Key_6,6);
	setPathKey(Qt::Key_7,7);
	setPathKey(Qt::Key_8,8);
	setPathKey(Qt::Key_9,9);


	currentCamDisplay = 0;
	graphDisplay = 0;


	setSnapshotFormat(QString("PNG"));

	reset();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void PointCloudViewer::setNodeHanlder(ros::NodeHandlePtr nh)
{
  nh_ = nh;
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/**
 * Destructor
 */
PointCloudViewer::~PointCloudViewer()
{
	delete currentCamDisplay;
	delete graphDisplay;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/**
 *
 */
void PointCloudViewer::reset()
{
	if(currentCamDisplay != 0)
		delete currentCamDisplay;
	if(graphDisplay != 0)
		delete graphDisplay;

	currentCamDisplay = new KeyFrameDisplay();
	graphDisplay = new KeyFrameGraphDisplay();


	resetRequested=false;

	save_folder = ros::package::getPath("lsd_slam_viewer")+"/save/";

	lastCamID = -1;
	lastCamTime = 0;

	setSceneRadius(80);
	setTextIsEnabled(false);




}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/**
 *
 * Note: this function is called from the main_viewer::frameCb callback function (runs every n Hz)
 */
void PointCloudViewer::addFrameMsg(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
	meddleMutex.lock();

	if(!msg->isKeyframe)
	{
		if(currentCamDisplay->id > msg->id)
		{
			printf("detected backward-jump in id (%d to %d), resetting!\n", currentCamDisplay->id, msg->id);
			resetRequested = true;
		}
		currentCamDisplay->setFrom(msg); /* KeyFrameDisplay */
		lastCamTime = msg->time;
		lastCamID = msg->id;
	}
	else
	{
		graphDisplay->addMsg(msg); /* KeyFrameGraphDisplay */

	}
	meddleMutex.unlock();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/**
 *
 * Note: this function is called from the main_viewer::graphCb callback function (runs every n Hz)
 */
void PointCloudViewer::addGraphMsg(lsd_slam_viewer::keyframeGraphMsgConstPtr msg)
{
	meddleMutex.lock();

	graphDisplay->addGraphMsg(msg); /* KeyFrameGraphDisplay */

	meddleMutex.unlock();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/**
 *
 */
void PointCloudViewer::init()
{
	setAnimationPeriod(30);
	startAnimation();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/**
 *
 */
QString PointCloudViewer::helpString() const
{
	return QString("Add something here!!!");
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/**
 * This function is a overload of the QGLViewer::draw() callback and runs at the frequency defined by
 * the QGLViewer::setAnimationPeriod() function (in this case 30 milliseconds)
 */
void PointCloudViewer::draw()
{
	meddleMutex.lock();

	if(resetRequested)
	{
		reset();
		resetRequested = false;
	}

	glPushMatrix();


	if(showCurrentCamera) // showCurrentCamera
		currentCamDisplay->drawCam(lineTesselation, 0);



  /* I don't know what this do, but there seems to be no influence not running this code, the cloud is drawn
   * any way when draw is called from graphDisplay */
	if(showCurrentPointcloud) // showCurrentPointcloud
		currentCamDisplay->drawPC(pointTesselation, 1);

	graphDisplay->draw(); // executes the code from the point clouds

	glPopMatrix();
	meddleMutex.unlock();

}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/**
 *
 */
void PointCloudViewer::keyReleaseEvent(QKeyEvent *e)
{
  //EMPTY
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/**
 *
 */
void PointCloudViewer::setToVideoSize()
{
	this->setFixedSize(1600,900);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/**
 * Handle events from the keyboard
 */
void PointCloudViewer::keyPressEvent(QKeyEvent *e)
  {
    switch (e->key())
    {
      /* ****************************************************************************************** */
      case Qt::Key_S :
        setToVideoSize();
    	break;

    	/* reset viewer ***************************************************************************** */
      case Qt::Key_R :
        resetRequested = true;
    	break;

    	/* ******************************************************************************************* */
      case Qt::Key_P:
    	  graphDisplay->flushPointcloud = true;
    	break;

    	/* ******************************************************************************************* */
      case Qt::Key_W:
    	  graphDisplay->printNumbers = true;
    	break;

    	/* ************************************************************************************************** */
      default:
    	  QGLViewer::keyPressEvent(e);
    	break;
    }
  }

//EOF
