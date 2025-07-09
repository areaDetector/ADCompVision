/**
 * NDPluginCV.cpp
 *
 * Top level source file for OpenCV based computer vision plugin for EPICS
 * Area Detector. Extends from the base NDPluginDriver found in ADCore, and
 * overrides its process callbacks function.
 *
 * All actual computer vision application is done in the helper file. In this .cpp file
 * there are functions that do the following:
 *
 * 1. Bidirectional conversion from OpenCV data type to NDDataType
 * 2. Bidirectional conversion from OpenCV color mode to NDColorMode
 * 3. Bidirectional conversion from OpenCV Mat image to NDArray
 * 4. PV Assignment functions for generic inputs and outputs
 * 5. Functions that call implementations from helper lib
 * 6. Function to call on updating PVs (writeInt32)
 * 7. Constructor/Destructor
 * 8. Plugin IOC shell registration
 *
 * Note that this file is intended to house all of the EPICS interfacing for the
 * ADCompVision plugin, and as a result, adding new OpenCV functionality should
 * NOT require any edits to be made to this file. Please read the CONTRIBUTING.md
 * file at the top level of this repository to learn how to add new functionality.
 *
 * Author: Jakub Wlodek
 *
 * Created: 23-Jun-2018
 * Last Updated: 09-Jul-2025
 * Copyright (c): Brookhaven National Laboratory 2018-2025
 */

// include some standard libraries
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>

// include epics/area detector libraries
#include <epicsExport.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <iocsh.h>

#include "NDArray.h"
#include "NDPluginCV.h"

// OpenCV is used for image manipulation
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

// some basic namespaces
using namespace std;
using namespace cv;

// name of the plugin
static const char* pluginName = "NDPluginCV";

//----------------------------------------------------------------------------
//-------------------- Data Type Conversion Functions ------------------------
//----------------------------------------------------------------------------

/**
 * Function that takes an NDDataType and an NDColorMode and returns an ADCVFrameFormat
 * The Frame Format type corresponds to the OpenCV data types. To see the mappings from
 * NDDataType/ColorMode to CV data type, check the CV_DTYP_CONV.md file, or look at the
 * website in the documentation
 *
 * @params[in]: dataType    -> dataType of NDArray
 * @params[in]: colorMode   -> colorMode of NDArray
 * @return: appropriate frame format based on dtype and color mode
 */
ADCVFrameFormat_t NDPluginCV::getCurrentImageFormat(NDDataType_t dataType,
                                                    NDColorMode_t colorMode) {
    const char* functionName = "getCurrentImageFormat";
    if (colorMode == NDColorModeMono) {
        switch (dataType) {
            case NDUInt8:
                return ADCV_Mono_U8;
            case NDInt8:
                return ADCV_Mono_U8;
            case NDUInt16:
                return ADCV_Mono_U16;
            case NDInt16:
                return ADCV_Mono_U16;
            case NDInt32:
                return ADCV_Mono_F32;
            case NDFloat32:
                return ADCV_Mono_F32;
            case NDFloat64:
                return ADCV_Mono_F64;
            default:
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                          "%s::%s Unsupported Image format\n", pluginName, functionName);
                return ADCV_UnsupportedFormat;
        }
    } else if (colorMode == NDColorModeRGB1) {
        switch (dataType) {
            case NDUInt8:
                return ADCV_RGB_U8;
            case NDInt8:
                return ADCV_RGB_U8;
            case NDUInt16:
                return ADCV_RGB_U16;
            case NDInt16:
                return ADCV_RGB_U16;
            case NDInt32:
                return ADCV_RGB_F32;
            case NDFloat32:
                return ADCV_RGB_F32;
            case NDFloat64:
                return ADCV_RGB_F64;
            default:
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                          "%s::%s Unsupported Image format\n", pluginName, functionName);
                return ADCV_UnsupportedFormat;
        }
    }
    return ADCV_UnsupportedFormat;
}

/**
 * Function that gets an NDDataType from an OpenCV Mat
 *
 * @params[in]:  matFormat   -> current image format of OpenCV Matrix image
 * @params[out]: pdataType   -> pointer to output data type
 * @return: status           -> asynSuccess if data Type identified, otherwise asynError
 */
asynStatus NDPluginCV::getDataTypeFromMat(ADCVDataFormat_t matFormat, NDDataType_t* pdataType) {
    const char* functionName = "getDataTypeFromMat";
    asynStatus status = asynSuccess;
    if (matFormat == ADCV_UnsupportedData) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Unsupported Image format\n",
                  pluginName, functionName);
        status = asynError;
    } else {
        if (matFormat == ADCV_U8)
            *pdataType = NDUInt8;
        else if (matFormat == ADCV_S8)
            *pdataType = NDInt8;
        else if (matFormat == ADCV_U16)
            *pdataType = NDUInt16;
        else if (matFormat == ADCV_S16)
            *pdataType = NDInt16;
        else if (matFormat == ADCV_S32)
            *pdataType = NDInt32;
        else if (matFormat == ADCV_F32)
            *pdataType = NDFloat32;
        else if (matFormat == ADCV_F64)
            *pdataType = NDFloat64;
        else
            status = asynError;
    }
    return status;
}

/**
 * Function that gets that NDColorMode from an OpenCV Mat
 *
 * @params[in]:  matFormat   -> current image format of OpenCV Matrix image
 * @params[out]: pcolorMode  -> pointer to output color mode
 * @return: status           -> asynSuccess if color mode identified, otherwise asynError
 */
asynStatus NDPluginCV::getColorModeFromMat(ADCVColorFormat_t matFormat, NDColorMode_t* pcolorMode) {
    const char* functionName = "getDataTypeFromMat";
    asynStatus status = asynSuccess;
    if (matFormat == ADCV_UnsupportedColor) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Unsupported Image format\n",
                  pluginName, functionName);
        status = asynError;
    } else {
        if (matFormat == ADCV_Mono)
            *pcolorMode = NDColorModeMono;
        else if (matFormat == ADCV_RGB)
            *pcolorMode = NDColorModeRGB1;
        else
            status = asynError;
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
 * @params[in]:  pArray      -> pointer to input array passed from the driver
 * @params[out]: pMat        -> Mat image taken from pArray
 * @params[in]:  dataType    -> dataType of current array
 * @params[in]:  colorMode   -> color mode of current array
 * @return: status -> success if converted correctly otherwise error
 */
asynStatus NDPluginCV::ndArray2Mat(NDArray* pArray, Mat& pMat, NDDataType_t dataType,
                                   NDColorMode_t colorMode) {
    const char* functionName = "ndArray2Mat";
    asynStatus status = asynSuccess;
    NDArrayInfo arrayInfo;
    // first get the matrix color format
    ADCVFrameFormat_t matFormat = getCurrentImageFormat(dataType, colorMode);

    if (matFormat == ADCV_UnsupportedFormat) {
        // if unsupported print error message
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Unsupported image format\n",
                  pluginName, functionName);
        status = asynError;
    } else {
        // otherwise generate an OpenCV mat of the appropriate size and insert the data
        pArray->getInfo(&arrayInfo);
        // set value at pointer
        // void* temp = malloc(arrayInfo.totalBytes);
        // memcpy(temp, pArray->pData, arrayInfo.totalBytes);
        Mat pMatTemp = Mat(arrayInfo.ySize, arrayInfo.xSize, matFormat, pArray->pData);
        pMat = pMatTemp.clone();
        pMatTemp.release();

        if (colorMode == NDColorModeRGB1) {
            // if it is color convert to BGR openCV functions use bgr as default
            cvtColor(pMat, pMat, COLOR_RGB2BGR);
        }
    }
    return status;
}

/*
 * Function that converts Mat object after it has been processed back to an NDArray for use in Area
 * Detector does not replace the original pArray, because otherwise
 *
 * @params[in]: pMat        -> pointer to opencv Mat object after processing
 * @params[out]: pScratch   -> Temporary NDArray that will be outputted by the plugin
 * @return: status -> success if converted correctly, otherwise error
 */
asynStatus NDPluginCV::mat2NDArray(Mat& pMat, NDArray* pScratch) {
    const char* functionName = "mat2NDArray";
    asynStatus status;
    unsigned char* dataStart = pMat.data;
    NDArrayInfo arrayInfo;
    pScratch->getInfo(&arrayInfo);
    // This way of finding the number of bytes in the mat must be used in case of possible
    // limitations on the number of bytes allowed by the system in a row of the image
    size_t dataSize = pMat.step[0] * pMat.rows;
    if (dataSize != arrayInfo.totalBytes) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s::%s Error converting from mat to NDArray\n", pluginName, functionName);
        status = asynError;
    } else {
        // copy image into NDArray
        memcpy((unsigned char*)pScratch->pData, dataStart, arrayInfo.totalBytes);
        // callParamCallbacks();
        // doCallbacksGenericPointer(pScratch, NDArrayData, 0);
        status = asynSuccess;
    }
    pMat.release();
    // pScratch->release();
    return status;
}

//----------------------------------------------------------------------------
//-------------------- Input and Output Management ---------------------------
//----------------------------------------------------------------------------

/**
 * Basic function used to assign input PV pointers into an array for easier iteration
 */
void NDPluginCV::assignInputs() {
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
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Finished assigning input values\n",
              pluginName, functionName);
}

/**
 * Basic function used to assign output PV pointers into an array for easier iteration
 */
void NDPluginCV::assignOutputs() {
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
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Finished assigning output values\n",
              pluginName, functionName);
}

/**
 * Basic function used to assign input Description PV pointers into an array for easier iteration
 */
void NDPluginCV::assignInputDescriptions() {
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
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s::%s Finished assigning input desc values\n", pluginName, functionName);
}

/**
 * Basic function used to assign output Description PV pointers into an array for easier iteration
 */
void NDPluginCV::assignOutputDescriptions() {
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
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s::%s Finished assigning output desc values\n", pluginName, functionName);
}

/**
 * Function that takes PV value from the plugin driver, and converts it into the ADCVFunction_t
 * enum type. This is used to decide which function the plugin is to perform as well
 * as to compute Input/Output descriptions
 *
 * @params[in]: pvValue         -> value of the PV when it is changed
 * @params[in]: functionSet     -> the set from which the function set came from. currently (1-3)
 * @return: function            -> returns the function as an ADCVFunction_t enum
 */
ADCVFunction_t NDPluginCV::get_function_from_pv(int pvValue, int functionSet) {
    const char* functionName = "get_function_from_pv";
    if (functionSet == 1) {
        return (ADCVFunction_t)pvValue;
    }
    if (functionSet == 2) {
        return (ADCVFunction_t)(N_FUNC_1 + pvValue - 1);
    }
    if (functionSet == 3) {
        return (ADCVFunction_t)(N_FUNC_1 + N_FUNC_2 + pvValue - 2);
    }
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s::%s ERROR: Couldn't find correct function val\n", pluginName, functionName);
    cvHelper->cvHelperStatus = "ERROR: Couldn't find correct function val";
    return ADCV_NoFunction;
}

/**
 * Function that checks whether the selected function can be processed on multiple threads in a safe
 * manner. Several functions use previous frames during processing and thus require external
 * variables. it is possible that in the future this will be fixed through mutexes/semaphores
 *
 * @params[in]: visionFunction -> the user selected vision function
 * @returns: true if it is not in the list of non thread safe functions, false otherwise
 */
bool NDPluginCV::isThreadSafe(ADCVFunction_t visionFunction) {
    bool threadSafe = true;
    int i;
    int len = (int)(sizeof(nonThreadSafeFunctions) / sizeof(ADCVFunction_t));
    for (i = 0; i < len; i++) {
        if (visionFunction == nonThreadSafeFunctions[i]) threadSafe = false;
    }
    return threadSafe;
}

/**
 * Function that pulls the input values from the PVs and puts them into an array
 *
 * @params[out]: inputs -> a pointer that is populated by the values stored in the input PVs.
 * @return: asynStatus
 */
asynStatus NDPluginCV::getRequiredParams(double* inputs) {
    const char* functionName = "getRequiredParams";
    asynStatus status = asynSuccess;
    int i;
    for (i = 0; i < NUM_INPUTS; i++) {
        status = getDoubleParam(inputPVs[i], (inputs + i));
        if (status == asynError) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s::%s Error reading back parameter values\n", pluginName, functionName);
            return status;
        }
    }
    return status;
}

/**
 * Function that pushes the values in the output array to PVs
 *
 * @params[in]: outputs -> a pointer to the outputs that are pushed to PVs
 * @return: asynStatus
 */
asynStatus NDPluginCV::setOutputParams(double* outputs) {
    const char* functionName = "setOutputParams";
    asynStatus status = asynSuccess;
    int i;
    for (i = 0; i < NUM_OUTPUTS; i++) {
        status = setDoubleParam(outputPVs[i], *(outputs + i));
        if (status == asynError) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s::%s Error reading back parameter values\n", pluginName, functionName);
            return status;
        }
    }
    // asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Could it be output params\n",
    // pluginName, functionName);
    callParamCallbacks();
    return status;
}

//----------------------------------------------------------------------------
//---------------- Functions that call CVHelper library ----------------------
//----------------------------------------------------------------------------

/**
 * Very simple function that allows for status information to be saved to a PV
 *
 * @params[in]: statusMessage -> new value for Status Message PV
 * @return: value returned by setStringParam
 */
asynStatus NDPluginCV::updatePluginStatus(string statusMessage) {
    asynStatus status = setStringParam(NDPluginCVStatusMessage, statusMessage);
    return status;
}

/*
 * Function calls the aproppriate image processing function. These functions can be
 * found in the helper .cpp file. To select the appropriate mode, it simply detects
 * the value in the PV CompVisionFunction_RBV which is set by the user. It switches on
 * this value, and calls the appropriate function, replacing the arguments in the call
 * with PV values, which can also be set by the user.
 *
 * @params[in]: inputImg    -> pointer to mat to be processed
 * @return: status      -> asynSuccess if success, asynError if OpenCV exception thrown
 */
asynStatus NDPluginCV::processImage(Mat& inputImg) {
    const char* functionName = "processImage";
    int functionSet1, functionSet2, functionSet3, cam_depth;
    asynStatus status = asynSuccess;
    ADCVStatus_t libStatus;

    // init arrays for inputs and outputs Adding some buffer room to maybe fix the realloc errors?
    double inputs[NUM_INPUTS];
    double outputs[NUM_OUTPUTS];

    // get the three functions
    getIntegerParam(NDPluginCVFunction1, &functionSet1);
    getIntegerParam(NDPluginCVFunction2, &functionSet2);
    getIntegerParam(NDPluginCVFunction3, &functionSet3);
    getIntegerParam(NDPluginCVCameraDepth, &cam_depth);

    ADCVCameraDepth_t camera_depth = (ADCVCameraDepth_t)cam_depth;
    ADCVFunction_t visionFunction;

    if (functionSet1 != 0) {
        visionFunction = get_function_from_pv(functionSet1, 1);
    } else if (functionSet2 != 0) {
        visionFunction = get_function_from_pv(functionSet2, 2);
    } else if (functionSet3 != 0) {
        visionFunction = get_function_from_pv(functionSet3, 3);
    } else {
        visionFunction = ADCV_NoFunction;
    }

    status = getRequiredParams(inputs);
    if (status != asynError && visionFunction != ADCV_NoFunction) {
        // if the vision function is thread safe, we can leave the mutex unlocked. If it isn't we
        // need to lock it back up For example, video recording and consecutive image subtraction
        // functions are not thread safe because they rely on NDPluginCVHelper classwide variables
        if (!isThreadSafe(visionFunction)) this->lock();

        libStatus = cvHelper->processImage(inputImg, visionFunction, camera_depth, inputs, outputs);

        if (!isThreadSafe(visionFunction)) this->unlock();

        if (libStatus == cvHelperError) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s::%s Error processing image in library\n", pluginName, functionName);
            status = asynError;
        } else if (libStatus == cvHelperWait) {
            status = asynTimeout;
        } else {
            status = setOutputParams(outputs);
        }
    }

    if (visionFunction == ADCV_NoFunction) {
        this->cvHelper->fix_coloration(inputImg);
        cvHelper->cvHelperStatus = "Image passed through";
        status = asynDisabled;
    }
    // writeImageFile(inputImg);
    updatePluginStatus(cvHelper->cvHelperStatus);
    return status;
}

/**
 * Function that is used to update the function descriptions for all of the input and
 * output PVS. Called every time the user switches to a different CV function
 *
 * @params[in]: function    -> function the user switched to
 * @return: asynSuccess if set correctly, otherwise asynError
 */
asynStatus NDPluginCV::updateFunctionDescriptions(ADCVFunction_t function) {
    const char* functionName = "updateFunctionDescriptions";
    string inputDesc[NUM_INPUTS];
    string outputDesc[NUM_OUTPUTS];
    string description;
    cvHelper->getFunctionDescription(function, inputDesc, outputDesc, &description);
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s::%s Updating I/O descriptions\n",
              pluginName, functionName);
    int k, l;
    for (k = 0; k < NUM_INPUTS; k++) {
        setStringParam(inputDescPVs[k], inputDesc[k]);
    }
    for (l = 0; l < NUM_INPUTS; l++) {
        setStringParam(outputDescPVs[l], outputDesc[l]);
    }
    setStringParam(NDPluginCVFunctionDescription, description);
    updatePluginStatus(cvHelper->cvHelperStatus);
    return asynSuccess;
}

/**
 * Checks if given file path is valid by simply attempting to open up a temp file in write mode in
 * the given path
 *
 * @params[in]: filepath    -> filepath entered by user
 * @return true if the path was valid, false otherwise
 */
bool NDPluginCV::checkFilepathValid(const char* filepath) {
    const char* functionName = "checkFilepathValid";
    // printf("%s::%s - %s\n", pluginName, functionName, filepath);
    char ext_buff[256];
    sprintf(ext_buff, "%s/%s", filepath, "/__NDCV_temp__");
    FILE* temp = fopen(ext_buff, "w");
    if (temp == NULL) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s::%s Inputted file path is invalid.\n",
                  pluginName, functionName);
        return false;
    }
    fclose(temp);
    return true;
}

//----------------------------------------------------------------------------
//---------------- Overwrites of NDPluginDriver Functions --------------------
//----------------------------------------------------------------------------

/**
 * Function that overwrites the NDPluginDriver function. This function is called when an octet PV is
 * changed in EPICS
 *
 * @params[in]: pasynUser       -> the thread/user that made the PV change
 * @params[in]: value           -> the value assigned to the PV
 * @params[in]: nChars          -> number of characters
 * @params[in]: nActual         -> actual number of bytes transferred
 * @return: status          -> success or failure
 */
asynStatus NDPluginCV::writeOctet(asynUser* pasynUser, const char* value, size_t nChars,
                                  size_t* nActual) {
    const char* functionName = "writeOctet";
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    int addr = 0;
    status = getAddress(pasynUser, &addr);
    if (status != asynSuccess) return (status);
    status = (asynStatus)setStringParam(function, (char*)value);
    if (status != asynSuccess) return (status);
    // printf("Entering writeOctet\n");
    if (function == NDFilePath) {
        char buff[256];
        getStringParam(NDFilePath, sizeof(buff), buff);
        bool check = checkFilepathValid(buff);
        if (check) {
            string input = buff;
            // printf("%s\n", input.c_str());
            this->cvHelper->update_str_in(&input);
            setIntegerParam(NDFilePathExists, 1);
        } else {
            setIntegerParam(NDFilePathExists, 0);
        }
    } else if (function < NDCV_FIRST_PARAM) {
        status = NDPluginDriver::writeOctet(pasynUser, value, nChars, nActual);
    }

    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Error writing Octet val to PV\n",
                  pluginName, functionName);
    } else
        asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s::%s function = %d value=%s\n",
                  pluginName, functionName, function, value);
    callParamCallbacks();
    *nActual = nChars;
    return status;
}

/**
 * Function that overwrites the NDPluginDriver function. This function is called when an integer PV
 * is changed in EPICS
 *
 * @params[in]: pasynUser       -> the thread/user that made the PV change
 * @params[in]: value           -> the value assigned to the PV
 * @return: status          -> success or failure
 */
asynStatus NDPluginCV::writeInt32(asynUser* pasynUser, epicsInt32 value) {
    const char* functionName = "writeInt32";
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;

    status = setIntegerParam(function, value);
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s::%s function=%d value=%d\n", pluginName,
              functionName, function, value);

    if (function == NDPluginCVFunction1 && value != 0) {
        setIntegerParam(NDPluginCVFunction2, 0);
        setIntegerParam(NDPluginCVFunction3, 0);
        ADCVFunction_t function = get_function_from_pv(value, 1);
        updateFunctionDescriptions(function);
    } else if (function == NDPluginCVFunction2 && value != 0) {
        setIntegerParam(NDPluginCVFunction1, 0);
        setIntegerParam(NDPluginCVFunction3, 0);
        ADCVFunction_t function = get_function_from_pv(value, 2);
        updateFunctionDescriptions(function);
    } else if (function == NDPluginCVFunction3 && value != 0) {
        setIntegerParam(NDPluginCVFunction1, 0);
        setIntegerParam(NDPluginCVFunction2, 0);
        ADCVFunction_t function = get_function_from_pv(value, 3);
        updateFunctionDescriptions(function);
    } else if ((function == NDPluginCVFunction1 || function == NDPluginCVFunction2 ||
                function == NDPluginCVFunction3) &&
               value == 0) {
        updateFunctionDescriptions(ADCV_NoFunction);
    } else if (function < NDCV_FIRST_PARAM) {
        // make sure to call base class for remaining PVs
        status = NDPluginDriver::writeInt32(pasynUser, value);
    }

    callParamCallbacks();
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s::%s ERROR status=%d, function=%d, value=%d\n", pluginName, functionName,
                  status, function, value);
        return asynError;
    } else
        asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s::%s function=%d value=%d\n",
                  pluginName, functionName, function, value);
    return asynSuccess;
}

/*
 * Function that overrides the process callbacks function in the base NDPluginDriver
 * class. This function recieves an Image in the form of an NDArray. Then the image is
 * converted into an OpenCV Mat. The processImage function is called on the Mat.
 * the resulting Mat is placed into a temporary NDArray which is then called back
 *
 * @params[in]: pArray -> pointer to image in the form of an NDArray
 * @return: void
 */
void NDPluginCV::processCallbacks(NDArray* pArray) {
    const char* functionName = "processCallbacks";

    asynStatus status;
    NDArrayInfo arrayInfo;

    pArray->getInfo(&arrayInfo);

    setIntegerParam(NDDataType, pArray->dataType);
    setIntegerParam(NDColorMode, arrayInfo.colorMode);

    // opencv mat for input
    Mat img;
    NDDataType_t finalDataType;
    NDColorMode_t finalColorMode;

    // copy the pArray into the mat
    status = ndArray2Mat(pArray, img, pArray->dataType, arrayInfo.colorMode);

    if (status == asynError) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s::%s Error copying from NDArray to Mat\n", pluginName, functionName);
    } else {
        NDPluginDriver::beginProcessCallbacks(pArray);

        // Function that calls on helper library - unlock for multi-thread processing
        this->unlock();
        status = processImage(img);
        this->lock();

        if (status == asynError) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Error Processing image\n",
                      pluginName, functionName);
            img.release();
            return;
        } else if (status == asynTimeout) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s::%s Waiting for next image\n",
                      pluginName, functionName);
            img.release();
            return;
        }

        // convert the data type and color format
        ADCVDataFormat_t matFormat = (ADCVDataFormat_t)img.depth();
        ADCVColorFormat_t colorFormat = (ADCVColorFormat_t)img.channels();
        status = getDataTypeFromMat(matFormat, &finalDataType);
        status = getColorModeFromMat(colorFormat, &finalColorMode);
        // copy from the output to the pScratch array
        // temp array so we don't overwrite the pArray passed to us from the camera
        int ndims;
        Size matSize = img.size();
        if (finalColorMode == NDColorModeMono)
            ndims = 2;
        else
            ndims = 3;
        size_t dims[ndims];
        if (ndims == 3) {
            dims[0] = 3;
            dims[1] = matSize.width;
            dims[2] = matSize.height;
        } else {
            dims[0] = matSize.width;
            dims[1] = matSize.height;
        }
        NDArray* pScratch = pNDArrayPool->alloc(ndims, dims, finalDataType, 0, NULL);
        if (pScratch == NULL) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s::%s Unable to allocate temp frame\n", pluginName, functionName);
            status = asynError;
            return;
        }
        status = mat2NDArray(img, pScratch);
        if (status == asynError) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s::%s Error copying from Mat to NDArray\n", pluginName, functionName);
            return;
        }

        pScratch->pAttributeList->add("ColorMode", "Color Mode", NDAttrInt32, &finalColorMode);
        pScratch->uniqueId = pArray->uniqueId;

        // refresh the PV values, and push the output image to NDArrayData
        NDPluginDriver::endProcessCallbacks(pScratch, false, true);
        callParamCallbacks();
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
 * @params[in]: portName            -> asyn port for the plugin
 * @params[in]: queueSize           -> number of Arrays the plugin can back up if it can't keep up
 * @params[in]: blockingCallbacks   -> Whether or not plugin can perform callbacks
 * @params[in]: NDArrayPort         -> portname for arrays sent out by the plugin
 * @params[in]: NDArrayAddr         -> address for arrays sent out by the plugin
 * @params[in]: maxBuffers          -> max buffer size for the plugin
 * @params[in]: maxMemeory          -> max memory the plugin can allocate
 * @params[in]: priority            -> plugin priority
 * @params[in]: stackSize           -> size of the stack given to the plugin
 */
NDPluginCV::NDPluginCV(const char* portName, int queueSize, int blockingCallbacks,
                       const char* NDArrayPort, int NDArrayAddr, int maxBuffers, size_t maxMemory,
                       int priority, int stackSize, int maxThreads)
    /* Invoke the base class constructor */
    : NDPluginDriver(portName, queueSize, blockingCallbacks, NDArrayPort, NDArrayAddr, 2,
                     maxBuffers, maxMemory,
                     asynInt32ArrayMask | asynFloat64ArrayMask | asynGenericPointerMask,
                     asynInt32ArrayMask | asynFloat64ArrayMask | asynGenericPointerMask, 0, 1,
                     priority, stackSize, maxThreads) {
    const char* functionName = "NDPluginCV";
    char versionString[25];

    // create function params (3)
    // more may need to be added, I'm not sure how many values an mbbo record supports
    createParam(NDPluginCVFunction1String, asynParamInt32, &NDPluginCVFunction1);
    createParam(NDPluginCVFunction2String, asynParamInt32, &NDPluginCVFunction2);
    createParam(NDPluginCVFunction3String, asynParamInt32, &NDPluginCVFunction3);

    // create the input params (10)
    createParam(NDPluginCVInput1String, asynParamFloat64, &NDPluginCVInput1);
    createParam(NDPluginCVInput2String, asynParamFloat64, &NDPluginCVInput2);
    createParam(NDPluginCVInput3String, asynParamFloat64, &NDPluginCVInput3);
    createParam(NDPluginCVInput4String, asynParamFloat64, &NDPluginCVInput4);
    createParam(NDPluginCVInput5String, asynParamFloat64, &NDPluginCVInput5);
    createParam(NDPluginCVInput6String, asynParamFloat64, &NDPluginCVInput6);
    createParam(NDPluginCVInput7String, asynParamFloat64, &NDPluginCVInput7);
    createParam(NDPluginCVInput8String, asynParamFloat64, &NDPluginCVInput8);
    createParam(NDPluginCVInput9String, asynParamFloat64, &NDPluginCVInput9);
    createParam(NDPluginCVInput10String, asynParamFloat64, &NDPluginCVInput10);

    // create the input description params (10)
    createParam(NDPluginCVInput1DescriptionString, asynParamOctet, &NDPluginCVInput1Description);
    createParam(NDPluginCVInput2DescriptionString, asynParamOctet, &NDPluginCVInput2Description);
    createParam(NDPluginCVInput3DescriptionString, asynParamOctet, &NDPluginCVInput3Description);
    createParam(NDPluginCVInput4DescriptionString, asynParamOctet, &NDPluginCVInput4Description);
    createParam(NDPluginCVInput5DescriptionString, asynParamOctet, &NDPluginCVInput5Description);
    createParam(NDPluginCVInput6DescriptionString, asynParamOctet, &NDPluginCVInput6Description);
    createParam(NDPluginCVInput7DescriptionString, asynParamOctet, &NDPluginCVInput7Description);
    createParam(NDPluginCVInput8DescriptionString, asynParamOctet, &NDPluginCVInput8Description);
    createParam(NDPluginCVInput9DescriptionString, asynParamOctet, &NDPluginCVInput9Description);
    createParam(NDPluginCVInput10DescriptionString, asynParamOctet, &NDPluginCVInput10Description);

    // create the float output params (3)
    createParam(NDPluginCVOutput1String, asynParamFloat64, &NDPluginCVOutput1);
    createParam(NDPluginCVOutput2String, asynParamFloat64, &NDPluginCVOutput2);
    createParam(NDPluginCVOutput3String, asynParamFloat64, &NDPluginCVOutput3);
    createParam(NDPluginCVOutput4String, asynParamFloat64, &NDPluginCVOutput4);
    createParam(NDPluginCVOutput5String, asynParamFloat64, &NDPluginCVOutput5);
    createParam(NDPluginCVOutput6String, asynParamFloat64, &NDPluginCVOutput6);
    createParam(NDPluginCVOutput7String, asynParamFloat64, &NDPluginCVOutput7);
    createParam(NDPluginCVOutput8String, asynParamFloat64, &NDPluginCVOutput8);
    createParam(NDPluginCVOutput9String, asynParamFloat64, &NDPluginCVOutput9);
    createParam(NDPluginCVOutput10String, asynParamFloat64, &NDPluginCVOutput10);

    // create the output description params (3)
    createParam(NDPluginCVOutput1DescriptionString, asynParamOctet, &NDPluginCVOutput1Description);
    createParam(NDPluginCVOutput2DescriptionString, asynParamOctet, &NDPluginCVOutput2Description);
    createParam(NDPluginCVOutput3DescriptionString, asynParamOctet, &NDPluginCVOutput3Description);
    createParam(NDPluginCVOutput4DescriptionString, asynParamOctet, &NDPluginCVOutput4Description);
    createParam(NDPluginCVOutput5DescriptionString, asynParamOctet, &NDPluginCVOutput5Description);
    createParam(NDPluginCVOutput6DescriptionString, asynParamOctet, &NDPluginCVOutput6Description);
    createParam(NDPluginCVOutput7DescriptionString, asynParamOctet, &NDPluginCVOutput7Description);
    createParam(NDPluginCVOutput8DescriptionString, asynParamOctet, &NDPluginCVOutput8Description);
    createParam(NDPluginCVOutput9DescriptionString, asynParamOctet, &NDPluginCVOutput9Description);
    createParam(NDPluginCVOutput10DescriptionString, asynParamOctet,
                &NDPluginCVOutput10Description);

    // createParam(NDPluginCVFilePathString,                       asynParamOctet,
    // &NDPluginCVFilePath); createParam(NDPluginCVPathExistsString, asynParamInt32,
    // &NDPluginCVPathExists);

    // createParam(NDPluginCVStringOutputString,                   asynParamOctet,
    // &NDPluginCVStringOutput); createParam(NDPluginCVStringOutputDescriptionString,
    // asynParamOctet,     &NDPluginCVStringOutputDescription);

    createParam(NDPluginCVCameraDepthString, asynParamInt32, &NDPluginCVCameraDepth);

    createParam(NDPluginCVFunctionDescriptionString, asynParamOctet,
                &NDPluginCVFunctionDescription);
    createParam(NDPluginCVStatusMessageString, asynParamOctet, &NDPluginCVStatusMessage);

    // assigns inputs and outputs to arrays to simplify iteration
    assignInputs();
    assignOutputs();
    assignInputDescriptions();
    assignOutputDescriptions();

    // init some plugin info
    setStringParam(NDPluginDriverPluginType, functionName);
    epicsSnprintf(versionString, sizeof(versionString), "%d.%d.%d", NDPluginCV_VERSION,
                  NDPluginCV_REVISION, NDPluginCV_MODIFICATION);
    setStringParam(NDDriverVersion, versionString);

    // This object will be the helper that will do the actual image processing
    cvHelper = new NDPluginCVHelper();

    connectToArrayPort();
}

/* NDPluginCV destructor, currently empty */
NDPluginCV::~NDPluginCV() { delete this->cvHelper; }

/* External function that is called in the IOC shell to create the plugin object */
extern "C" int NDCVConfigure(const char* portName, int queueSize, int blockingCallbacks,
                             const char* NDArrayPort, int NDArrayAddr, int maxBuffers,
                             size_t maxMemory, int priority, int stackSize, int maxThreads) {
    // calls the plugin constructor
    NDPluginCV* pPlugin =
        new NDPluginCV(portName, queueSize, blockingCallbacks, NDArrayPort, NDArrayAddr, maxBuffers,
                       maxMemory, priority, stackSize, maxThreads);

    // starts the plugin
    return pPlugin->start();
}

/* IOC shell argument initialization here */
static const iocshArg initArg0 = {"portName", iocshArgString};
static const iocshArg initArg1 = {"frame queue size", iocshArgInt};
static const iocshArg initArg2 = {"blocking callbacks", iocshArgInt};
static const iocshArg initArg3 = {"NDArrayPort", iocshArgString};
static const iocshArg initArg4 = {"NDArrayAddr", iocshArgInt};
static const iocshArg initArg5 = {"maxBuffers", iocshArgInt};
static const iocshArg initArg6 = {"maxMemory", iocshArgInt};
static const iocshArg initArg7 = {"priority", iocshArgInt};
static const iocshArg initArg8 = {"stackSize", iocshArgInt};
static const iocshArg initArg9 = {"maxThreads", iocshArgInt};
static const iocshArg* const initArgs[] = {&initArg0, &initArg1, &initArg2, &initArg3, &initArg4,
                                           &initArg5, &initArg6, &initArg7, &initArg8, &initArg9};

/* defines the configuration function for Initializing the plugin */
static const iocshFuncDef initFuncDef = {"NDCVConfigure", 10, initArgs};

/* Init call function for the IOC shell */
static void initCallFunc(const iocshArgBuf* args) {
    NDCVConfigure(args[0].sval, args[1].ival, args[2].ival, args[3].sval, args[4].ival,
                  args[5].ival, args[6].ival, args[7].ival, args[8].ival, args[9].ival);
}

/* Registration of NDPluginCV for PV autosaving and into the IOC shell command set*/
extern "C" void NDCVRegister(void) { iocshRegister(&initFuncDef, initCallFunc); }

/* Extern C function called from IOC startup script */
extern "C" {
epicsExportRegistrar(NDCVRegister);
}
