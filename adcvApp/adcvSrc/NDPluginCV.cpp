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


/*
 * Function that converts incoming NDArray into an OpenCV Mat that will be passed to the 
 * image processing functions. First, we copy the original NDArray into a scratch array,
 * so that we do not affect the performance of othe plugins. Next, the function generates
 * a blank Mat object, and then finally copies the data from the NDArray into the Mat.
 * 
 * @params: pScratch -> pointer to a blank temporary NDArray
 * @params: pArray -> NDArray recieved from the camera
 * @params: numCols -> number of columns in the original NDArray
 * @params: numRows -> number of rows in the original NDArray
 * @return: the converted OpenCV Mat
 */
Mat NDPluginCV::getMatFromNDArray(NDArray* pScratch, NDArray* pArray, int numCols, int numRows){
    NDimension_t scratch_dims[2];
    unsigned char *inData, *outData;
    pScratch->initDimension(&scratch_dims[0], numCols);
    pScratch->initDimension(&scratch_dims[1], numRows);
    this->pNDArrayPool->convert(pArray, &pScratch, NDUInt8);
    int numRowsScratch, numColsScratch;
    NDArrayInfo scratchInfo;
    pScratch->getInfo(&scratchInfo);
    numRowsScratch = pScratch->dims[scratchInfo.yDim].size;
    numColsScratch = pScratch->dims[scratchInfo.xDim].size;
    Mat img = Mat(numRowsScratch, numColsScratch, CV_8UC1);
    inData = (unsigned char*) pScratch->pData;
    outData = (unsigned char*) img.data;
    memcpy(outData, inData, scratchInfo.nElements*sizeof(unsigned char));
    return img;
}

/*
 * Function calls the aproppriate image processing function. These functions can be
 * found in the helper .cpp file. To select the appropriate mode, it simply detects
 * the value in the PV CompVisionFunction_RBV which is set by the user. It switches on
 * this value, and calls the appropriate function, replacing the arguments in the call
 * with PV values, which can also be set by the user.
 * 
 * @params: visionMode -> value in CompVisionFunction_RBV PV
 * @params: img -> image to be processed
 * @return: void
 */
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
            int roiX, roiY, roiWidth, roiHeight, blurDegree, threshVal;
            getIntegerParam(NDPluginCVROICornerX, &roiX);
            getIntegerParam(NDPluginCVROICornerY, &roiY);
            getIntegerParam(NDPluginCVROIWidth, &roiWidth);
            getIntegerParam(NDPluginCVROIHeight, &roiHeight);
            getIntegerParam(NDPluginCVThresholdVal, &threshVal);
            getIntegerParam(NDPluginCVBlurDegree, &blurDegree);
            Mat result = centroid_finder(img, roiX, roiY, roiWidth, roiHeight, blurDegree, threshVal);
            break;
        default :
            asynPrint(this->pasynUserSelf, "%s::%s No valid image processing selected.\n", driverName, functionName);
            break;
    }
}

/*
 * Function that overrides the process callbacks function in the base NDPluginDriver
 * class. This function recieves an Image in the form of an NDArray. Then it checks
 * if the image is in mono form. Then, it converts it into an OpenCV Mat. Finally,
 * it calls the processImage function to perform the desired computer vision operation
 * 
 * @params: pArray -> pointer to image in the form of an NDArray
 * @return: void
 */
void NDPluginCV::processCallbacks(NDArray *pArray){
    NDArray* pScratch = NULL;
    NDArrayInfo arrayInfo;
    unsigned int numRows, numCols;
    

    static const char* functionName = "processCallbacks";

    if(pArray.ndims !=2){
        asynPrint(this->pasynUserSelf, "%s::%s Please convert image passed to image processing plugin to mono\n", diriverName, functionName);
        return;
    }
    NDPluginDriver::beginProcessCallbacks(pArray);

    pArray->getInfo(&arrayInfo);
    numCols = pArray->dims[arrayInfo.xDim].size;
    numRows = pArray->dims[arrayInfo.yDim].size;

    this->unlock();

    Mat img = getMatFromNDArray(pScratch, pArray);
    int visionMode;
    getIntegerParam(NDPluginCVVisionFunction, &visionMode);
    processImage(visionMode, img);
    this->lock();
    if(NULL != pScratch){
        pScratch->release();
    }
    callParamCallbacks();
}

NDPluginCV::NDPluginCV(const char *portName, int queueSize, int blockingCallbacks,
		    const char *NDArrayPort, int NDArrayAddr,
		    int maxBuffers, size_t maxMemory,
		    int priority, int stackSize)
		    /* Invoke the base class constructor */
		    : NDPluginDriver(portName, queueSize, blockingCallbacks,
		    NDArrayPort, NDArrayAddr, 1, maxBuffers, maxMemory,
		    asynInt32ArrayMask | asynFloat64ArrayMask | asynGenericPointerMask,
		    asynInt32ArrayMask | asynFloat64ArrayMask | asynGenericPointerMask,
            ASYN_MULTIDEVICE, 1, priority, stackSize, 1)
{
    char versionString[25];

    //create the parameters
    createParam(NDPluginCVComputerVisionFunctionString, asynParamInt32, &NDPluginCVComputerVisionFunction);
    createParam(NDPluginCVThresholdValueString, asynParamFloat64, &NDPluginCVThresholdValue);
    createParam(NDPluginCVThresholdRatioString, asynParamFloat64, &NDPluginCVThresholdRatio);
    createParam(NDPluginCVBlurDegreeString, asynParamInt32, &NDPluginCVBlurDegree);
    createParam(NDPluginCVEdgeMethodString, asynParamInt32, &NDPluginCVEdgeMethod);
    createParam(NDPluginCVROICornerXString, asynParamInt32, &NDPluginCVROICornerX);
    createParam(NDPluginCVROICornerYString, asynParamInt32, &NDPluginCVROICornerY);
    createParam(NDPluginCVROIWidthString, asynParamInt32, &NDPluginCVROIWidth);
    createParam(NDPluginCVROIHeightString, asynParamInt32, &NDPluginCVROIHeight);

    setStringParam(NDPluginDriverPluginType, "NDPluginCV");
    epicsSprintf(versionString, sizof(versionString), "%d.%d.%d", NDPluginCV_VERSION, NDPluginCV_REVISION, NDPluginCV_MODIFICATION);
    setStringParam(NDDriverVersion, versionString);
    connectToArrayPort();
}


extern "C" int NDCVConfigure(const char *portName, int queueSize, int blockingCallbacks,
	const char *NDArrayPort, int NDArrayAddr,
	int maxBuffers, size_t maxMemory,
	int priority, int stackSize){
	    NDPluginBar *pPlugin = new NDPluginBar(portName, queueSize, blockingCallbacks, NDArrayPort, NDArrayAddr,
	        maxBuffers, maxMemory, priority, stackSize);
	        return pPlugin->start();
}

static const iocshArg initArg0 = { "portName",iocshArgString};
static const iocshArg initArg1 = { "frame queue size",iocshArgInt};
static const iocshArg initArg2 = { "blocking callbacks",iocshArgInt};
static const iocshArg initArg3 = { "NDArrayPort",iocshArgString};
static const iocshArg initArg4 = { "NDArrayAddr",iocshArgInt};
static const iocshArg initArg5 = { "maxBuffers",iocshArgInt};
static const iocshArg initArg6 = { "maxMemory",iocshArgInt};
static const iocshArg initArg7 = { "priority",iocshArgInt};
static const iocshArg initArg8 = { "stackSize",iocshArgInt};
static const iocshArg * const initArgs[] = {&initArg0,
					&initArg1,
					&initArg2,
					&initArg3,
					&initArg4,
					&initArg5,
					&initArg6,
					&initArg7,
					&initArg8};

static const iocshFuncDef initFuncDef = {"NDCVConfigure",9,initArgs};


static void initCallFunc(const iocshArgBuf *args){
	NDCVConfigure(args[0].sval, args[1].ival, args[2].ival,
			args[3].sval, args[4].ival, args[5].ival,
			args[6].ival, args[7].ival, args[8].ival);
}


extern "C" void NDCVRegister(void){
	iocshRegister(&initFuncDef,initCallFunc);
}

extern "C" {
	epicsExportRegistrar(NDCVRegister);
}