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
#include "NDPluginCVHelper.h"
#include <epicsExport.h>

//OpenCV is used for image manipulation
#include <opencv2/opencv.hpp>

//some basic namespaces
using namespace std;
using namespace cv;
static const char *driverName="NDPluginCV";


void NDPluginCV::processImage(int visionMode, Mat &img){

    static char* functionName = "processImage";

    switch(visionMode){
        case 0 :
            int edgeDet;
            getIntegerParam(NDPluginCVEdgeMethod, &edgeDet);
            if(edgeDet == 0){
                int threshVal, threshRatio, blurDegree;
                getIntegerParam(NDPluginCVThresholdVal, &threshVal);
                getIntegerParam(NDPluginCVThresholdRatio, &threshRatio);
                getIntegerParam(NDPluginCVBlurDegree, &blurDegree);
                Mat result = edge_detector_canny(img, threshVal, threshRatio, blurDegree);
                break;
            }
            else if(edgeDet == 1){
                int blurDegree;
                getIntegerParam(NDPluginCVBlurDegree, &blurDegree);
                Mat result = edge_detector_laplacian(img, blurDegree);
                break;
            }
            else asynPrint(this->pasynUserSelf, "%s::%s No valid edge detector selected\n", driverName, functionName);
        case 1 :
            //TODO
            break;
        default :
            asynPrint(this->pasynUserSelf, "%s::%s No valid image processing selected.\n", driverName, functionName);
            break;
    }
}


void NDPluginCV::processCallbacks(NDArray *pArray){
    NDArray* pScratch = NULL;
    NDArrayInfo arrayInfo;
    unsigned int numRows, numCols;
    unsigned char *inData, *outData;

    static const char* functionName = "processCallbacks";
}