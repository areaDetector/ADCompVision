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
static const char *pluginName="NDPluginCV";

/*

TODO (NOW)

* Rewrite the ndArray2Mat function to support all kinds of NDArray Data types
* Write a mat2NDArray funtion for converting back into NDArray
* remove all the 'wrapper' functions, I will do this diffferently
* Rewrite the process image funciton to simply call a function from the helper lib
* have all necessary params as global vars that are passed as pointer


TODO (LATER)

* Allow for chainging CV functions by passing int* and num_processes to helper lib

*/


/**
 * Function that takes an NDDataType and an NDColorMode and returns an ADCVFrameFormat
 * The Frame Format type corresponds to the OpenCV data types. To see the mappings from 
 * NDDataType/ColorMode to CV data type, check the CV_DTYP_CONV.md file, or look at the 
 * website in the documentation
 * 
 * @params: dataType -> dataType of NDArray
 * @params: colorMode -> colorMode of NDArray
 * @return: appropriate frame format based on dtype and color mode
 */
ADCVFrameFormat_t NDPluginCV::getCurrentImageFormat(NDDataType_t dataType, NDColorMode_t colorMode){
    static const char* functionName = "getCurrentImageFormat";
    if(colorMode==NDColorModeMono){
        switch(dataType){
            case NDUInt8:
                return ADCV_Mono_U8;
            case NDInt8:
                return ADCV_Mono_S8;
            case NDUInt16:
                return ADCV_Mono_U16;
            case NDInt16:
                return ADCV_Mono_S16;
            case NDInt32:
                return ADCV_Mono_S32;
            case NDFloat32:
                return ADCV_Mono_F32;
            case NDFloat64:
                return ADCV_Mono_F64;
            default:
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Unsupported Image format\n", pluginName, functionName);
                return ADCV_UnsupportedFormat;
        }
    }
    else if(colorMode == NDColorModeRGB1){
        switch(dataType){
            case NDUInt8:
                return ADCV_RGB_U8;
            case NDInt8:
                return ADCV_RGB_S8;
            case NDUInt16:
                return ADCV_RGB_U16;
            case NDInt16:
                return ADCV_RGB_S16;
            case NDInt32:
                return ADCV_RGB_S32;
            case NDFloat32:
                return ADCV_RGB_F32;
            case NDFloat64:
                return ADCV_RGB_F64;
            default:
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Unsupported Image format\n", pluginName, functionName);
                return ADCV_UnsupportedFormat;
        }
    }
}

/**
 * Function that gets that NDDataType from an OpenCV Mat
 * 
 * @params: matFormat -> current image format of OpenCV Matrix image
 * @params: pdataType -> pointer to output data type
 * @return: status -> asynSuccess if data Type identified, otherwise asynError
 */
asynStatus NDPluginCV::getDataTypeFromMat(ADCVFrameFormat_t matFormat, NDDataType_t* pdataType){
    static const char* functionName = "getDataTypeFromMat";
    asynStatus status = asynSuccess;
    if(matFormat == ADCV_UnsupportedFormat){
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Unsupported Image format\n", pluginName, functionName);
        status = asynError;
    }
    else{
        if(matFormat == ADCV_Mono_U8 || matFormat == ADCV_RGB_U8) *pdataType = NDUInt8;
        else if(matFormat == ADCV_Mono_S8 || matFormat == ADCV_RGB_S8) *pdataType = NDInt8;
        else if(matFormat == ADCV_Mono_U16 || matFormat == ADCV_RGB_U16) *pdataType = NDUInt16;
        else if(matFormat == ADCV_Mono_S16 || matFormat == ADCV_RGB_S16) *pdataType = NDInt16;
        else if(matFormat == ADCV_Mono_S32 || matFormat == ADCV_RGB_S32) *pdataType = NDInt32;
        else if(matFormat == ADCV_Mono_F32 || matFormat == ADCV_RGB_F32) *pdataType = NDFloat32;
        else if(matFormat == ADCV_Mono_F64 || matFormat == ADCV_RGB_F64) *pdataType = NDFloat64;
        else status = asynError;
    }
    return status;
}


Mat ndArray2Mat(NDArray* pArray, NDDataType_t dataType, NDColorMode_t colorMode){

}



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
    NDDimension_t scratch_dims[2];
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
 * Wrapper function for canny. Gets args from PV and calls helper function
 */
Mat NDPluginCV::canny_wrapper(Mat &img){
    int threshVal, threshRatio, blurDegree;
    getIntegerParam(NDPluginCVThresholdValue, &threshVal);
    getIntegerParam(NDPluginCVThresholdRatio, &threshRatio);
    getIntegerParam(NDPluginCVBlurDegree, &blurDegree);
    Mat result = cvHelper->edge_detector_canny(img, threshVal, threshRatio, blurDegree);
    return result;
}

/*
 * Wrapper function for laplacian. Gets args from PV and calls helper function
 */
Mat NDPluginCV::laplacian_wrapper(Mat &img){
    int blurDegree;
    getIntegerParam(NDPluginCVBlurDegree, &blurDegree);
    Mat result = cvHelper->edge_detector_laplacian(img, blurDegree);
    return result;
}

/*
 * Wrapper function for centroid finding. Gets args from PV and calls helper function
 */
Mat NDPluginCV::centroid_wrapper(Mat &img){
    int roiX, roiY, roiWidth, roiHeight, blurDegree, threshVal;
    getIntegerParam(NDPluginCVROICornerX, &roiX);
    getIntegerParam(NDPluginCVROICornerY, &roiY);
    getIntegerParam(NDPluginCVROIWidth, &roiWidth);
    getIntegerParam(NDPluginCVROIHeight, &roiHeight);
    getIntegerParam(NDPluginCVThresholdValue, &threshVal);
    getIntegerParam(NDPluginCVBlurDegree, &blurDegree);
    Mat result = cvHelper->centroid_finder(img, roiX, roiY, roiWidth, roiHeight, blurDegree, threshVal);
    return result;
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

    static const char* functionName = "processImage";
    Mat result;

    switch(visionMode){
        case 0 :
            int edgeDet;
            getIntegerParam(NDPluginCVEdgeMethod, &edgeDet);
            if(edgeDet == 0){
               result = canny_wrapper(img);
                break;
            }
            else if(edgeDet == 1){
                result = laplacian_wrapper(img);
                break;
            }
            else asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s No valid edge detector selected\n", pluginName, functionName);
        case 1 :
            result = centroid_wrapper(img);
            break;
        default :
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s No valid image processing selected.\n", pluginName, functionName);
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

    if(pArray->ndims !=2){
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Please convert image passed to image processing plugin to mono\n", pluginName, functionName);
        return;
    }
    NDPluginDriver::beginProcessCallbacks(pArray);

    pArray->getInfo(&arrayInfo);
    numCols = pArray->dims[arrayInfo.xDim].size;
    numRows = pArray->dims[arrayInfo.yDim].size;

    this->unlock();

    Mat img = getMatFromNDArray(pScratch, pArray, numCols, numRows);
    int visionMode;
    getIntegerParam(NDPluginCVComputerVisionFunction, &visionMode);
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
    epicsSnprintf(versionString, sizeof(versionString), "%d.%d.%d", NDPluginCV_VERSION, NDPluginCV_REVISION, NDPluginCV_MODIFICATION);
    setStringParam(NDDriverVersion, versionString);

    cvHelper = new NDPluginCVHelper();

    connectToArrayPort();
}


/* NDPluginCV destructor, currently empty */
NDPluginCV::~NDPluginCV(){ }


extern "C" int NDCVConfigure(const char *portName, int queueSize, int blockingCallbacks,
	const char *NDArrayPort, int NDArrayAddr,
	int maxBuffers, size_t maxMemory,
	int priority, int stackSize){
	    NDPluginCV *pPlugin = new NDPluginCV(portName, queueSize, blockingCallbacks, NDArrayPort, NDArrayAddr,
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
