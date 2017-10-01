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

#include "settings.h"




float pointTesselation = 1;
float lineTesselation = 4;

bool keepInMemory=false;
bool showKFCameras = false;
bool showKFPointclouds = false;
bool showConstraints = false;
bool showCurrentCamera = false;
bool showCurrentPointcloud = false;

float scaledDepthVarTH = 1;
float absDepthVarTH = 1;
int minNearSupport = 5;
int cutFirstNKf = 5;
int sparsifyFactor = 1;

int memorySize = 15;
int numberOfSampler = 10;

int numRefreshedAlready = 0;

// cut-off after this
double lastFrameTime = 1e15;

