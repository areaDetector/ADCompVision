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
#include <opencv2/core/core.hpp>

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
 * @params: dataType    -> dataType of NDArray
 * @params: colorMode   -> colorMode of NDArray
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
 * @params: matFormat   -> current image format of OpenCV Matrix image
 * @params: pdataType   -> pointer to output data type
 * @return: status      -> asynSuccess if data Type identified, otherwise asynError
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

/**
 * Function that gets that NDColorMode from an OpenCV Mat
 * 
 * @params: matFormat   -> current image format of OpenCV Matrix image
 * @params: pcolorMode  -> pointer to output color mode
 * @return: status      -> asynSuccess if color mode identified, otherwise asynError
 */
asynStatus NDPluginCV::getColorModeFromMat(ADCVFrameFormat_t matFormat, NDColorMode_t* pcolorMode){
    static const char* functionName = "getDataTypeFromMat";
    asynStatus status = asynSuccess;
    if(matFormat == ADCV_UnsupportedFormat){
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Unsupported Image format\n", pluginName, functionName);
        status = asynError;
    }
    else{
        if(matFormat == ADCV_Mono_U8 || matFormat == ADCV_Mono_S8 ||
            matFormat == ADCV_Mono_U16 || matFormat == ADCV_Mono_S16 ||
            matFormat == ADCV_Mono_S32 || matFormat == ADCV_Mono_F32 ||
            matFormat == ADCV_Mono_F64)
                *pcolorMode = NDColorModeMono;
        else if(matFormat == ADCV_RGB_U8 || matFormat == ADCV_RGB_S8 ||
            matFormat == ADCV_RGB_U16 || matFormat == ADCV_RGB_S16 ||
            matFormat == ADCV_RGB_S32 || matFormat == ADCV_RGB_F32 ||
            matFormat == ADCV_RGB_F64)
                *pcolorMode = NDColorModeRGB1;
        else status = asynError;
    }
    return status;
}


/**
 * Function that will take a pointer to an NDArray, and converts it into
 * an OpenCV "Mat" image object
 * 
 * @params: pArray      -> pointer to input array passed from the driver
 * @params: dataType    -> dataType of current array
 * @params: colorMode   -> color mode of current array
 * 
 */
asynStatus NDPluginCV::ndArray2Mat(NDArray* pArray, Mat* pMat, NDDataType_t dataType, NDColorMode_t colorMode){
    static const char* functionName = "ndArray2Mat";
    asynStatus status = asynSuccess;
    NDArrayInfo arrayInfo;
    //first get the matrix color format
    ADCVFrameFormat_t matFormat = getCurrentImageFormat(dataType, colorMode);
    if(matFormat == ADCV_UnsupportedFormat){
        //if unsupported print error message
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Unsupported image format\n", pluginName, functionName);
        status = asynError;
    }
    else{
        //otherwise generate an OpenCV mat of the appropriate size and insert the data
        pArray->getInfo(&arrayInfo);
        //set value at pointer
        *pMat = Mat(arrayInfo.ySize, arrayInfo.xSize, matFormat, pArray->pData);
        if(colorMode == NDColorModeRGB1){
            //if it is color convert to BGR openCV functions use bgr as default
            cvtColor(*pMat, *pMat, COLOR_RGB2BGR);
        }
    }
    return status;
}


/*
 * Function that converts Mat object after it has been processed back to an NDArray for use in Area
 * Detector does not replace the original pArray, because otherwise 
 *
 * @params: pScratch -> pointer to a blank temporary NDArray
 * @params: pMat -> pointer to opencv Mat object after processing
 */
asynStatus NDPluginCV::mat2NDArray(NDArray* pScratch, Mat* pMat){
    static const char* functionName = "mat2NDArray";
    asynStatus status;
    int ndims;
    NDDataType_t dataType;
    NDColorMode_t colorMode;
    Size matSize = pMat->size();
    ADCVFrameFormat_t matFormat = (ADCVFrameFormat_t) pMat->depth();
    status = getDataTypeFromMat(matFormat, &dataType);
    if(status == asynError) return status;
    status = getColorModeFromMat(matFormat, &colorMode);
    if(status == asynError) return status;
    if(colorMode == NDColorModeMono){
        ndims = 2;
    }
    else{
        ndims = 3;
    }
    size_t dims[ndims];
    if(ndims == 3){
        dims[0] == pMat->channels();
        dims[1] == matSize.width;
        dims[2] == matSize.height;
    }
    else{
        dims[0] == matSize.width;
        dims[1] == matSize.height;
    }
    pScratch = pNDArrayPool->alloc(ndims, dims, dataType, 0, NULL);
    if(pScratch == NULL){
        pScratch->release();
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Unable to allocate temp frame\n", pluginName, functionName);
        status = asynError;
    }
    else{
        unsigned char* dataStart = pMat->data;
        // This way of finding the number of bytes in the mat must be used in case of possible limitations on the 
        // number of bytes allowed by the system in a row of the image
        int dataSize = pMat->step[0] * pMat->rows;
        //copy image into NDArray
        memcpy(pScratch->pData, dataStart, dataSize);
        pScratch->pAttributeList->add("ColorMode", "Color Mode", NDAttrInt32, &colorMode);
        pScratch->pAttributeList->add("DataType", "Data Type", NDAttrInt32, &dataType);
        status = asynSuccess;
    }
    return status;
}


/*
 * Function calls the aproppriate image processing function. These functions can be
 * found in the helper .cpp file. To select the appropriate mode, it simply detects
 * the value in the PV CompVisionFunction_RBV which is set by the user. It switches on
 * this value, and calls the appropriate function, replacing the arguments in the call
 * with PV values, which can also be set by the user.
 * 
 * @params: visionMode  -> value in CompVisionFunction_RBV PV
 * @params: inputImg    -> pointer to mat to be processed
 * @params: outputImg   -> pointer to output of image processing
 * @return: status      -> asynSuccess if success, asynError if OpenCV exception thrown
 */
asynStatus NDPluginCV::processImage(int visionMode, Mat* inputImg, Mat* outputImg){
    static const char* functionName = "processImage";
    //TODO



}

/*
 * Function that overrides the process callbacks function in the base NDPluginDriver
 * class. This function recieves an Image in the form of an NDArray. Then the image is
 * converted into an OpenCV Mat. The processImage function is called on the Mat.
 * the resulting Mat is placed into a temporary NDArray which is then called back
 *
 * @params: pArray -> pointer to image in the form of an NDArray
 * @return: void
 */
void NDPluginCV::processCallbacks(NDArray *pArray){
    static const char* functionName = "processCallbacks";
    asynStatus status;
    // temp array so we don't overwrite the pArray passed to us from the camera
    NDArray* pScratch;
    int dataType;
    int colorMode;
    getIntegerParam(NDDataType, &dataType);
    getIntegerParam(NDColorMode, &colorMode);
    //opencv mats for input and output
    Mat inputImage;
    Mat outputImage;
    // copy the pArray into the mat
    status = ndArray2Mat(pArray, &inputImage, (NDDataType_t) dataType, (NDColorMode_t) colorMode);
    
    //test to see if copying function works
    imshow("Test image", inputImage);
    waitKey(1);

    NDPluginDriver::beginProcessCallbacks(pArray);

    // do the computations on multiple threads
    this->unlock();



    this->lock();

    callParamCallbacks();
    getAttributes(pScratch->pAttributeList);
    doCallbacksGenericPointer(pScratch, NDArrayData, 0);

    pScratch->release();
}



/**
 * Constructor for NDPluginCV. Most parameters are passed on to the superclass
 * NDPluginDriver constructor. Next, PV parameters are initialized, and then the
 * plugin connects to the new Array port.
 * 
 * @params: portName            -> asyn port for the plugin
 * @params: queueSize           -> number of Arrays the plugin can back up if it can't keep up
 * @params: blockingCallbacks   -> Whether or not plugin can perform callbacks
 * @params: NDArrayPort         -> portname for arrays sent out by the plugin
 * @params: NDArrayAddr         -> address for arrays sent out by the plugin
 * @params: maxBuffers          -> max buffer size for the plugin
 * @params: maxMemeory          -> max memory the plugin can allocate
 * @params: priority            -> plugin priority
 * @params: stackSize           -> size of the stack given to the plugin
 */
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

    //create function params (3)
    createParam(NDPluginCVFunction1String,          asynParamInt32,     &NDPluginCVFunction1);
    createParam(NDPluginCVFunction2String,          asynParamInt32,     &NDPluginCVFunction2);
    createParam(NDPluginCVFunction3String,          asynParamInt32,     &NDPluginCVFunction3);

    //create the float input params (5)
    createParam(NDPluginCVFloatInput1String,        asynParamFloat64,   &NDPluginCVFloatInput1);
    createParam(NDPluginCVFloatInput2String,        asynParamFloat64,   &NDPluginCVFloatInput2);
    createParam(NDPluginCVFloatInput3String,        asynParamFloat64,   &NDPluginCVFloatInput3);
    createParam(NDPluginCVFloatInput4String,        asynParamFloat64,   &NDPluginCVFloatInput4);
    createParam(NDPluginCVFloatInput5String,        asynParamFloat64,   &NDPluginCVFloatInput5);

    //create the integer input params (5)
    createParam(NDPluginCVIntegerInput1String,      asynParamInt32,     &NDPluginCVIntegerInput1);
    createParam(NDPluginCVIntegerInput2String,      asynParamInt32,     &NDPluginCVIntegerInput2);
    createParam(NDPluginCVIntegerInput3String,      asynParamInt32,     &NDPluginCVIntegerInput3);
    createParam(NDPluginCVIntegerInput4String,      asynParamInt32,     &NDPluginCVIntegerInput4);
    createParam(NDPluginCVIntegerInput5String,      asynParamInt32,     &NDPluginCVIntegerInput5);

    //create the float output params (3)
    createParam(NDPluginCVFloatOutput1String,       asynParamFloat64,   &NDPluginCVFloatOutput1);
    createParam(NDPluginCVFloatOutput2String,       asynParamFloat64,   &NDPluginCVFloatOutput2);
    createParam(NDPluginCVFloatOutput3String,       asynParamFloat64,   &NDPluginCVFloatOutput3);

    //create the integer output params (3)
    createParam(NDPluginCVIntegerOutput1String,     asynParamInt32,     &NDPluginCVIntegerOutput1);
    createParam(NDPluginCVIntegerOutput2String,     asynParamInt32,     &NDPluginCVIntegerOutput2);
    createParam(NDPluginCVIntegerOutput3String,     asynParamInt32,     &NDPluginCVIntegerOutput3);

    //create the remaining params
    createParam(NDPluginCVOutputDescriptionString,  asynParamOctet, &NDPluginCVOutputDescription);

    setStringParam(NDPluginDriverPluginType, "NDPluginCV");
    epicsSnprintf(versionString, sizeof(versionString), "%d.%d.%d", NDPluginCV_VERSION, NDPluginCV_REVISION, NDPluginCV_MODIFICATION);
    setStringParam(NDDriverVersion, versionString);

    // This object will be the helper that will do the actual image processing 
    cvHelper = new NDPluginCVHelper();

    connectToArrayPort();
}


/* NDPluginCV destructor, currently empty */
NDPluginCV::~NDPluginCV(){ }


extern "C" int NDCVConfigure(const char *portName, int queueSize, int blockingCallbacks, const char *NDArrayPort, 
    int NDArrayAddr, int maxBuffers, size_t maxMemory, int priority, int stackSize){
	
    // calls the plugin constructor
    NDPluginCV *pPlugin = new NDPluginCV(portName, queueSize, blockingCallbacks, NDArrayPort, NDArrayAddr,
	    maxBuffers, maxMemory, priority, stackSize);
	
    // starts the plugin
    return pPlugin->start();
}

/* IOC shell argument initialization here */
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


/* defines the configuration function for Initializing the plugin */
static const iocshFuncDef initFuncDef = {"NDCVConfigure",9,initArgs};


/* Init call function for the IOC shell */
static void initCallFunc(const iocshArgBuf *args){
	NDCVConfigure(args[0].sval, args[1].ival, args[2].ival,
			args[3].sval, args[4].ival, args[5].ival,
			args[6].ival, args[7].ival, args[8].ival);
}

/* Registration of NDPluginCV for PV autosaving and into the IOC shell command set*/
extern "C" void NDCVRegister(void){
	iocshRegister(&initFuncDef, initCallFunc);
}

extern "C" {
	epicsExportRegistrar(NDCVRegister);
}
