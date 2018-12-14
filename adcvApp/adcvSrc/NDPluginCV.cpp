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

// name of the plugin
static const char *pluginName="NDPluginCV";


//----------------------------------------------------------------------------
//-------------------- Data Type Conversion Functions ------------------------
//----------------------------------------------------------------------------


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
    const char* functionName = "getCurrentImageFormat";
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
    return ADCV_UnsupportedFormat;
}


/**
 * Function that gets that NDDataType from an OpenCV Mat
 * 
 * @params: matFormat   -> current image format of OpenCV Matrix image
 * @params: pdataType   -> pointer to output data type
 * @return: status      -> asynSuccess if data Type identified, otherwise asynError
 */
asynStatus NDPluginCV::getDataTypeFromMat(ADCVDataFormat_t matFormat, NDDataType_t* pdataType){
    const char* functionName = "getDataTypeFromMat";
    asynStatus status = asynSuccess;
    if(matFormat == ADCV_UnsupportedData){
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Unsupported Image format\n", pluginName, functionName);
        status = asynError;
    }
    else{
        if(matFormat == ADCV_U8) *pdataType = NDUInt8;
        else if(matFormat == ADCV_S8) *pdataType = NDInt8;
        else if(matFormat == ADCV_U16) *pdataType = NDUInt16;
        else if(matFormat == ADCV_S16) *pdataType = NDInt16;
        else if(matFormat == ADCV_S32) *pdataType = NDInt32;
        else if(matFormat == ADCV_F32) *pdataType = NDFloat32;
        else if(matFormat == ADCV_F64) *pdataType = NDFloat64;
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
asynStatus NDPluginCV::getColorModeFromMat(ADCVColorFormat_t matFormat, NDColorMode_t* pcolorMode){
    const char* functionName = "getDataTypeFromMat";
    asynStatus status = asynSuccess;
    if(matFormat == ADCV_UnsupportedColor){
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Unsupported Image format\n", pluginName, functionName);
        status = asynError;
    }
    else{
        if(matFormat == ADCV_Mono) *pcolorMode = NDColorModeMono;
        else if(matFormat == ADCV_RGB) *pcolorMode = NDColorModeRGB1;
        else status = asynError;
    }
    return status;
}


//----------------------------------------------------------------------------
//---------------- Conversion to/from Mat and NDArray ------------------------
//----------------------------------------------------------------------------


/**
 * Function that will take a pointer to an NDArray, and converts it into
 * an OpenCV "Mat" image object
 * 
 * @params: pArray      -> pointer to input array passed from the driver
 * @params: dataType    -> dataType of current array
 * @params: colorMode   -> color mode of current array
 * 
 */
asynStatus NDPluginCV::ndArray2Mat(NDArray* pArray, Mat &pMat, NDDataType_t dataType, NDColorMode_t colorMode){
    const char* functionName = "ndArray2Mat";
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
        pMat = Mat(arrayInfo.ySize, arrayInfo.xSize, matFormat, pArray->pData);
        if(colorMode == NDColorModeRGB1){
            //if it is color convert to BGR openCV functions use bgr as default
            cvtColor(pMat, pMat, COLOR_RGB2BGR);
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
asynStatus NDPluginCV::mat2NDArray(NDArray* pScratch, Mat &pMat, NDDataType_t dataType, NDColorMode_t colorMode){
    const char* functionName = "mat2NDArray";
    asynStatus status;
    //asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s converting into pscratch\n", pluginName, functionName);
    unsigned char* dataStart = pMat.data;
    NDArrayInfo arrayInfo;
    pScratch->getInfo(&arrayInfo);
    // This way of finding the number of bytes in the mat must be used in case of possible limitations on the 
    // number of bytes allowed by the system in a row of the image
    size_t dataSize = pMat.step[0] * pMat.rows;
    if(dataSize != arrayInfo.totalBytes){
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Error converting from mat to NDArray\n", pluginName, functionName);
        status = asynError;
    }
    else{
        //copy image into NDArray
        memcpy((unsigned char*) pScratch->pData, dataStart, arrayInfo.totalBytes);
        pScratch->pAttributeList->add("ColorMode", "Color Mode", NDAttrInt32, &colorMode);
        pScratch->pAttributeList->add("DataType", "Data Type", NDAttrInt32, &dataType);
        getAttributes(pScratch->pAttributeList);
        status = asynSuccess;
    }
    return status;
}


//----------------------------------------------------------------------------
//-------------------- Input and Output Management ---------------------------
//----------------------------------------------------------------------------


/**
 * Basic function used to assign input PV pointers into an array for easier iteration
 */
void NDPluginCV::assignInputs(){
    const char* functionName = "assignInputs";
    inputPVs[0] = NDPluginCVInput1;
    inputPVs[1] = NDPluginCVInput2;
    inputPVs[2] = NDPluginCVInput3;
    inputPVs[3] = NDPluginCVInput4;
    inputPVs[4] = NDPluginCVInput5;
    inputPVs[5] = NDPluginCVInput6;
    inputPVs[6] = NDPluginCVInput7;
    inputPVs[7] = NDPluginCVInput8;
    inputPVs[8] = NDPluginCVInput9;
    inputPVs[9] = NDPluginCVInput10;
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Finished assigning input values\n", pluginName, functionName);
}


/**
 * Basic function used to assign output PV pointers into an array for easier iteration
 */
void NDPluginCV::assignOutputs(){
    const char* functionName = "assignOutputs";
    outputPVs[0] = NDPluginCVOutput1;
    outputPVs[1] = NDPluginCVOutput2;
    outputPVs[2] = NDPluginCVOutput3;
    outputPVs[3] = NDPluginCVOutput4;
    outputPVs[4] = NDPluginCVOutput5;
    outputPVs[5] = NDPluginCVOutput6;
    outputPVs[6] = NDPluginCVOutput7;
    outputPVs[7] = NDPluginCVOutput8;
    outputPVs[8] = NDPluginCVOutput9;
    outputPVs[9] = NDPluginCVOutput10;
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Finished assigning output values\n", pluginName, functionName);
}

/**
 * Basic function used to assign input Description PV pointers into an array for easier iteration
 */
void NDPluginCV::assignInputDescriptions(){
    const char* functionName = "assignInputDescriptions";
    inputDescPVs[0] = NDPluginCVInput1Description;
    inputDescPVs[1] = NDPluginCVInput2Description;
    inputDescPVs[2] = NDPluginCVInput3Description;
    inputDescPVs[3] = NDPluginCVInput4Description;
    inputDescPVs[4] = NDPluginCVInput5Description;
    inputDescPVs[5] = NDPluginCVInput6Description;
    inputDescPVs[6] = NDPluginCVInput7Description;
    inputDescPVs[7] = NDPluginCVInput8Description;
    inputDescPVs[8] = NDPluginCVInput9Description;
    inputDescPVs[9] = NDPluginCVInput10Description;
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Finished assigning input desc values\n", pluginName, functionName);
}


/**
 * Basic function used to assign output Description PV pointers into an array for easier iteration
 */
void NDPluginCV::assignOutputDescriptions(){
    const char* functionName = "assignOutputDescriptions";
    outputDescPVs[0] = NDPluginCVOutput1Description;
    outputDescPVs[1] = NDPluginCVOutput2Description;
    outputDescPVs[2] = NDPluginCVOutput3Description;
    outputDescPVs[3] = NDPluginCVOutput4Description;
    outputDescPVs[4] = NDPluginCVOutput5Description;
    outputDescPVs[5] = NDPluginCVOutput6Description;
    outputDescPVs[6] = NDPluginCVOutput7Description;
    outputDescPVs[7] = NDPluginCVOutput8Description;
    outputDescPVs[8] = NDPluginCVOutput9Description;
    outputDescPVs[9] = NDPluginCVOutput10Description;
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Finished assigning output desc values\n", pluginName, functionName);
}


/**
 * Function that pulls the input values from the PVs and puts them into an array
 * 
 * @params[out]: inputs -> a pointer that is populated by the values stored in the input PVs.
 * @return: asynStatus
 */
asynStatus NDPluginCV::getRequiredParams(double* inputs){
    const char* functionName = "getRequiredParams";
    asynStatus status = asynSuccess;
    int i;
    for(i=0; i<NUM_INPUTS; i++){
        status = getDoubleParam(inputPVs[i], (inputs+i));
        if(status == asynError){
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Error reading back parameter values\n", pluginName, functionName);
            return status;
        }   
    }
    return status;
}


/**
 * Function that pulls the ouptu values from the PVs and puts them into an array
 * 
 * @params[in]: outputs -> a pointer to the outputs that are pushed to PVs
 * @return: asynStatus
 */
asynStatus NDPluginCV::setOutputParams(double* outputs){
    const char* functionName = "setOutputParams";
    asynStatus status = asynSuccess;
    int i;
    for(i=0; i<NUM_OUTPUTS; i++){
        status = setDoubleParam(outputPVs[i], *(outputs+i));
        if(status == asynError){
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Error reading back parameter values\n", pluginName, functionName);
            return status;
        }  
    }
    //asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Could it be output params\n", pluginName, functionName);
    callParamCallbacks();
    return status;
}


//----------------------------------------------------------------------------
//---------------- Functions that call CVHelper library ----------------------
//----------------------------------------------------------------------------


/*
 * Function calls the aproppriate image processing function. These functions can be
 * found in the helper .cpp file. To select the appropriate mode, it simply detects
 * the value in the PV CompVisionFunction_RBV which is set by the user. It switches on
 * this value, and calls the appropriate function, replacing the arguments in the call
 * with PV values, which can also be set by the user.
 * 
 * @params: inputImg    -> pointer to mat to be processed
 * @params: outputImg   -> pointer to output of image processing
 * @return: status      -> asynSuccess if success, asynError if OpenCV exception thrown
 */
asynStatus NDPluginCV::processImage(Mat &inputImg){
    const char* functionName = "processImage";
    int functionSet1, functionSet2, functionSet3;
    asynStatus status = asynSuccess;
    ADCVStatus_t libStatus;

    // init arrays for inputs and outputs
    double* inputs = (double*) calloc(1, NUM_INPUTS*sizeof(double));
    double* outputs = (double*) calloc(1, NUM_OUTPUTS*sizeof(double));

    // get the three functions
    getIntegerParam(NDPluginCVFunction1, &functionSet1);
    getIntegerParam(NDPluginCVFunction2, &functionSet2);
    getIntegerParam(NDPluginCVFunction3, &functionSet3);

    ADCVFunction_t visionFunction;

    if(functionSet1 != 0){
        visionFunction = cvHelper->get_function_from_pv(functionSet1, 1);
    }
    else if(functionSet2 != 0){
        visionFunction = cvHelper->get_function_from_pv(functionSet2, 2);
    }
    else if(functionSet3 != 0){
        visionFunction = cvHelper->get_function_from_pv(functionSet3, 3);
    }
    else{
        visionFunction = ADCV_NoFunction;
    }

    if(visionFunction != ADCV_NoFunction){
        status = getRequiredParams(inputs);
        if(status != asynError){
            libStatus = cvHelper->processImage(inputImg, visionFunction, inputs, outputs);
            if(libStatus == cvHelperError){
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Error processing image in library\n", pluginName, functionName);
                status  = asynError;
            }
            else{
                status = setOutputParams(outputs);
            }
        }
    }
    else{
        status = asynError;
    }
    return status;
}


asynStatus NDPluginCV::updateFunctionDescriptions(ADCVFunction_t function){
    const char* functionName = "updateFunctionDescriptions";
    ADCVStatus_t status;
    string inputDesc[NUM_INPUTS];
    string outputDesc[NUM_OUTPUTS];
    string description;
    status = cvHelper->getFunctionDescription(function, inputDesc, outputDesc, &description);

    int k, l;
    for(k = 0; k< NUM_INPUTS; k++){
        setStringParam(inputDescPVs[k], inputDesc[k]);
    }
    for(l = 0; l< NUM_INPUTS; l++){
        setStringParam(outputDescPVs[l], outputDesc[l]);
    }
    setStringParam(NDPluginCVFunctionDescription, description);
    return asynSuccess;
}


//----------------------------------------------------------------------------
//---------------- Overwrites of NDPluginDriver Functions --------------------
//----------------------------------------------------------------------------


/**
 * Function that overwrites the NDPluginDriver function. This function is called when an integer PV is
 * changed in EPICS
 * 
 * @params: pasynUser       -> the thread/user that made the PV change
 * @params: value           -> the value assigned to the PV
 * @return: status          -> success or failure
 */
asynStatus NDPluginCV::writeInt32(asynUser* pasynUser, epicsInt32 value){
    const char* functionName = "writeInt32";
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;

    status = setIntegerParam(function, value);
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s function=%d value=%d\n", pluginName, functionName, function, value);

    if(function == NDPluginCVFunction1 && value != 0){
        setIntegerParam(NDPluginCVFunction2, 0);
        setIntegerParam(NDPluginCVFunction3, 0);
        ADCVFunction_t function = cvHelper->get_function_from_pv(value, 1);
        updateFunctionDescriptions(function);
    }
    else if(function == NDPluginCVFunction2 && value != 0){
        setIntegerParam(NDPluginCVFunction1, 0);
        setIntegerParam(NDPluginCVFunction3, 0);
        ADCVFunction_t function = cvHelper->get_function_from_pv(value, 2);
        updateFunctionDescriptions(function);

    }
    else if(function == NDPluginCVFunction3 && value != 0){
        setIntegerParam(NDPluginCVFunction1, 0);
        setIntegerParam(NDPluginCVFunction2, 0);
        ADCVFunction_t function = cvHelper->get_function_from_pv(value, 3);
        updateFunctionDescriptions(function);

    }
    else if(function < NDCV_FIRST_PARAM){
        //make sure to call base class for remaining PVs
        status = NDPluginDriver::writeInt32(pasynUser, value);
    }

    callParamCallbacks();
    if(status){
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s ERROR status=%d, function=%d, value=%d\n", pluginName, functionName, status, function, value);
        return asynError;
    }
    else asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s function=%d value=%d\n", pluginName, functionName, function, value);
    return asynSuccess;
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
    const char* functionName = "processCallbacks";

    if(firstFrame == 0){
        firstFrame = 1;
        return;
    }

    asynStatus status;
    NDArrayInfo arrayInfo;
    // temp array so we don't overwrite the pArray passed to us from the camera
    NDArray* pScratch;

    pArray->getInfo(&arrayInfo);

    setIntegerParam(NDDataType, pArray->dataType);
    setIntegerParam(NDColorMode, arrayInfo.colorMode);

    //opencv mat for input
    Mat img;
    NDDataType_t finalDataType;
    NDColorMode_t finalColorMode;

    // copy the pArray into the mat
    status = ndArray2Mat(pArray, img, pArray->dataType, arrayInfo.colorMode);

    if(status == asynError){
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Error copying from NDArray to Mat\n", pluginName, functionName);
    }
    else{
        //test to see if copying function works
        //imshow("Test image", inputImage);
        //waitKey(1);

        NDPluginDriver::beginProcessCallbacks(pArray);

        // do the computations on multiple threads
        this->unlock();

        // Function that calls on helper library
        status = processImage(img);
        
        this->lock();

        if(status == asynError){
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Error Processing image\n", pluginName, functionName);
            img.release();
        }
        else{
            //asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Could it be the lock\n", pluginName, functionName);
            // prepare the output NDArray pScratch
            int ndims;
            Size matSize = img.size();
            ADCVDataFormat_t matFormat = (ADCVDataFormat_t) img.depth();
            ADCVColorFormat_t colorFormat = (ADCVColorFormat_t) img.channels();
            status = getDataTypeFromMat(matFormat, &finalDataType);
            status = getColorModeFromMat(colorFormat, &finalColorMode);
            if(finalColorMode == NDColorModeMono){
                ndims = 2;
            }
            else{
                ndims = 3;
            }
            size_t dims[ndims];
            if(ndims == 3){
                dims[0] = img.channels();
                dims[1] = matSize.width;
                dims[2] = matSize.height;
            }
            else{
                dims[0] = matSize.width;
                dims[1] = matSize.height;
            }
            pScratch = pNDArrayPool->alloc(ndims, dims, finalDataType, 0, NULL);
            if(pScratch == NULL){
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Unable to allocate temp frame\n", pluginName, functionName);
                status = asynError;
            }

            // copy from the output to the pScratch array
            status = mat2NDArray(pScratch, img, finalDataType, finalColorMode);

            if(status == asynError){
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Error copying from Mat to NDArray\n", pluginName, functionName);
                //pScratch->release();
            }

            // refresh the PV values, and push the output image to NDArrayData. then release the memory for pScratch
            callParamCallbacks();
            doCallbacksGenericPointer(pScratch, NDArrayData, 0);
            img.release();
            pScratch->release();
        }
    }
}


//----------------------------------------------------------------------------
//---------------- Constructor/Destructor for NDPluginCV ---------------------
//----------------------------------------------------------------------------


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
    const char* functionName = "NDPluginCV";
    char versionString[25];

    //create function params (3) 
    //more may need to be added, I'm not sure how many values an mbbo record supports
    createParam(NDPluginCVFunction1String,                      asynParamInt32,     &NDPluginCVFunction1);
    createParam(NDPluginCVFunction2String,                      asynParamInt32,     &NDPluginCVFunction2);
    createParam(NDPluginCVFunction3String,                      asynParamInt32,     &NDPluginCVFunction3);

    //create the input params (10)
    createParam(NDPluginCVInput1String,                         asynParamFloat64,   &NDPluginCVInput1);
    createParam(NDPluginCVInput2String,                         asynParamFloat64,   &NDPluginCVInput2);
    createParam(NDPluginCVInput3String,                         asynParamFloat64,   &NDPluginCVInput3);
    createParam(NDPluginCVInput4String,                         asynParamFloat64,   &NDPluginCVInput4);
    createParam(NDPluginCVInput5String,                         asynParamFloat64,   &NDPluginCVInput5);
    createParam(NDPluginCVInput6String,                         asynParamFloat64,   &NDPluginCVInput6);
    createParam(NDPluginCVInput7String,                         asynParamFloat64,   &NDPluginCVInput7);
    createParam(NDPluginCVInput8String,                         asynParamFloat64,   &NDPluginCVInput8);
    createParam(NDPluginCVInput9String,                         asynParamFloat64,   &NDPluginCVInput9);
    createParam(NDPluginCVInput10String,                        asynParamFloat64,   &NDPluginCVInput10);

    //create the input description params (10)
    createParam(NDPluginCVInput1DescriptionString,              asynParamOctet,     &NDPluginCVInput1Description);
    createParam(NDPluginCVInput2DescriptionString,              asynParamOctet,     &NDPluginCVInput2Description);
    createParam(NDPluginCVInput3DescriptionString,              asynParamOctet,     &NDPluginCVInput3Description);
    createParam(NDPluginCVInput4DescriptionString,              asynParamOctet,     &NDPluginCVInput4Description);
    createParam(NDPluginCVInput5DescriptionString,              asynParamOctet,     &NDPluginCVInput5Description);
    createParam(NDPluginCVInput6DescriptionString,              asynParamOctet,     &NDPluginCVInput6Description);
    createParam(NDPluginCVInput7DescriptionString,              asynParamOctet,     &NDPluginCVInput7Description);
    createParam(NDPluginCVInput8DescriptionString,              asynParamOctet,     &NDPluginCVInput8Description);
    createParam(NDPluginCVInput9DescriptionString,              asynParamOctet,     &NDPluginCVInput9Description);
    createParam(NDPluginCVInput10DescriptionString,             asynParamOctet,     &NDPluginCVInput10Description);

    //create the float output params (3)
    createParam(NDPluginCVOutput1String,                        asynParamFloat64,   &NDPluginCVOutput1);
    createParam(NDPluginCVOutput2String,                        asynParamFloat64,   &NDPluginCVOutput2);
    createParam(NDPluginCVOutput3String,                        asynParamFloat64,   &NDPluginCVOutput3);
    createParam(NDPluginCVOutput4String,                        asynParamFloat64,   &NDPluginCVOutput4);
    createParam(NDPluginCVOutput5String,                        asynParamFloat64,   &NDPluginCVOutput5);
    createParam(NDPluginCVOutput6String,                        asynParamFloat64,   &NDPluginCVOutput6);
    createParam(NDPluginCVOutput7String,                        asynParamFloat64,   &NDPluginCVOutput7);
    createParam(NDPluginCVOutput8String,                        asynParamFloat64,   &NDPluginCVOutput8);
    createParam(NDPluginCVOutput9String,                        asynParamFloat64,   &NDPluginCVOutput9);
    createParam(NDPluginCVOutput10String,                       asynParamFloat64,   &NDPluginCVOutput10);

    //create the output description params (3)
    createParam(NDPluginCVOutput1DescriptionString,             asynParamOctet,     &NDPluginCVOutput1Description);
    createParam(NDPluginCVOutput2DescriptionString,             asynParamOctet,     &NDPluginCVOutput2Description);
    createParam(NDPluginCVOutput3DescriptionString,             asynParamOctet,     &NDPluginCVOutput3Description);
    createParam(NDPluginCVOutput4DescriptionString,             asynParamOctet,     &NDPluginCVOutput4Description);
    createParam(NDPluginCVOutput5DescriptionString,             asynParamOctet,     &NDPluginCVOutput5Description);
    createParam(NDPluginCVOutput6DescriptionString,             asynParamOctet,     &NDPluginCVOutput6Description);
    createParam(NDPluginCVOutput7DescriptionString,             asynParamOctet,     &NDPluginCVOutput7Description);
    createParam(NDPluginCVOutput8DescriptionString,             asynParamOctet,     &NDPluginCVOutput8Description);
    createParam(NDPluginCVOutput9DescriptionString,             asynParamOctet,     &NDPluginCVOutput9Description);
    createParam(NDPluginCVOutput10DescriptionString,            asynParamOctet,     &NDPluginCVOutput10Description);

    createParam(NDPluginCVFunctionDescriptionString,            asynParamOctet,     &NDPluginCVFunctionDescription);


    // assigns inputs and outputs to arrays to simplify iteration
    assignInputs();
    assignOutputs();
    assignInputDescriptions();
    assignOutputDescriptions();

    setStringParam(NDPluginDriverPluginType, functionName);
    epicsSnprintf(versionString, sizeof(versionString), "%d.%d.%d", NDPluginCV_VERSION, NDPluginCV_REVISION, NDPluginCV_MODIFICATION);
    setStringParam(NDDriverVersion, versionString);

    // This object will be the helper that will do the actual image processing 
    cvHelper = new NDPluginCVHelper();

    connectToArrayPort();
}


/* NDPluginCV destructor, currently empty */
NDPluginCV::~NDPluginCV(){ }


/* External function that is called in the IOC shell to create the plugin object */
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
