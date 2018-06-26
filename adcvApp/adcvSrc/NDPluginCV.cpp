/*
 * NDPluginCV.cpp
 * 
 * Top level source file for OpenCV based computer vision plugin for EPICS
 * Area Detector. Extends from the base NDPluginDriver found in ADCore, and
 * overrides its process callbacks function.
 * The OpenCV computer vision library is used for all image processing
 * 
 * Author: Jakub Wlodek
 * 
 * Created: June 23, 2018
 * 
 */


//include some standard libraries
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <stdio.h>

//include epics/area detector libraries
#include <epicsMutex.h>
#include <epicsString.h>
#include <iocsh.h>
#include "NDArray.h"
#include "NDPluginCV.h"
#include <epicsExport.h>

//OpenCV is used for image manipulation
#include <opencv2/opencv.hpp>

//some basic namespaces
using namespace std;
using namespace cv;
static const char *driverName="NDPluginCV";