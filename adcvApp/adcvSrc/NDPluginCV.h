/**
 * NDPluginCV.h
 * 
 * Main header file for NDPluginCV
 *
 * Includes version number definition, the class itself, function definitions,
 * and parameter definitions. The parameters themselves are initialized
 * in the constructor in the corresponding .cpp file
 *
 * Author: Jakub Wlodek
 *
 * Created: 26-Jun-18
 * Last Updated: 14-Jan-2019
 * Copyright (c): Brookhaven National Laboratory 2018-2019
 */

//include guard to avoid multiple inclusions
#ifndef NDPluginCV_H
#define NDPluginCV_H

#include <opencv2/opencv.hpp>

// standard and opencv namespaces used
using namespace std;
using namespace cv;

// base driver header file
#include "NDPluginDriver.h"

// Header file that contains CV Wrapper functions
#include "NDPluginCVHelper.h"

// version numbers
#define NDPluginCV_VERSION          1
#define NDPluginCV_REVISION         0
#define NDPluginCV_MODIFICATION     0

/* Definitions of parameters */

// CV functions (3)
#define NDPluginCVFunction1String                   "NDCV_FUNCTION1"            //asynInt32
#define NDPluginCVFunction2String                   "NDCV_FUNCTION2"            //asynInt32
#define NDPluginCVFunction3String                   "NDCV_FUNCTION3"            //asynInt32

// Input PVs (10)
#define NDPluginCVInput1String                      "NDCV_IN1"                  //asynFloat64
#define NDPluginCVInput2String                      "NDCV_IN2"                  //asynFloat64
#define NDPluginCVInput3String                      "NDCV_IN3"                  //asynFloat64
#define NDPluginCVInput4String                      "NDCV_IN4"                  //asynFloat64
#define NDPluginCVInput5String                      "NDCV_IN5"                  //asynFloat64
#define NDPluginCVInput6String                      "NDCV_IN6"                  //asynFloat64
#define NDPluginCVInput7String                      "NDCV_IN7"                  //asynFloat64
#define NDPluginCVInput8String                      "NDCV_IN8"                  //asynFloat64
#define NDPluginCVInput9String                      "NDCV_IN9"                  //asynFloat64
#define NDPluginCVInput10String                     "NDCV_IN10"                 //asynFloat64

// Input Description PVs (10)
#define NDPluginCVInput1DescriptionString           "NDCV_IN_DESCRIPTION1"      //asynParamOctet
#define NDPluginCVInput2DescriptionString           "NDCV_IN_DESCRIPTION2"      //asynParamOctet
#define NDPluginCVInput3DescriptionString           "NDCV_IN_DESCRIPTION3"      //asynParamOctet
#define NDPluginCVInput4DescriptionString           "NDCV_IN_DESCRIPTION4"      //asynParamOctet
#define NDPluginCVInput5DescriptionString           "NDCV_IN_DESCRIPTION5"      //asynParamOctet
#define NDPluginCVInput6DescriptionString           "NDCV_IN_DESCRIPTION6"      //asynParamOctet
#define NDPluginCVInput7DescriptionString           "NDCV_IN_DESCRIPTION7"      //asynParamOctet
#define NDPluginCVInput8DescriptionString           "NDCV_IN_DESCRIPTION8"      //asynParamOctet
#define NDPluginCVInput9DescriptionString           "NDCV_IN_DESCRIPTION9"      //asynParamOctet
#define NDPluginCVInput10DescriptionString          "NDCV_IN_DESCRIPTION10"     //asynParamOctet

// Output PVs (10)
#define NDPluginCVOutput1String                     "NDCV_OUT1"                 //asynFloat64
#define NDPluginCVOutput2String                     "NDCV_OUT2"                 //asynFloat64
#define NDPluginCVOutput3String                     "NDCV_OUT3"                 //asynFloat64
#define NDPluginCVOutput4String                     "NDCV_OUT4"                 //asynFloat64
#define NDPluginCVOutput5String                     "NDCV_OUT5"                 //asynFloat64
#define NDPluginCVOutput6String                     "NDCV_OUT6"                 //asynFloat64
#define NDPluginCVOutput7String                     "NDCV_OUT7"                 //asynFloat64
#define NDPluginCVOutput8String                     "NDCV_OUT8"                 //asynFloat64
#define NDPluginCVOutput9String                     "NDCV_OUT9"                 //asynFloat64
#define NDPluginCVOutput10String                    "NDCV_OUT10"                //asynFloat64

// Output Description PVs (10)
#define NDPluginCVOutput1DescriptionString          "NDCV_OUT_DESCRIPTION1"     //asynParamOctet
#define NDPluginCVOutput2DescriptionString          "NDCV_OUT_DESCRIPTION2"     //asynParamOctet
#define NDPluginCVOutput3DescriptionString          "NDCV_OUT_DESCRIPTION3"     //asynParamOctet
#define NDPluginCVOutput4DescriptionString          "NDCV_OUT_DESCRIPTION4"     //asynParamOctet
#define NDPluginCVOutput5DescriptionString          "NDCV_OUT_DESCRIPTION5"     //asynParamOctet
#define NDPluginCVOutput6DescriptionString          "NDCV_OUT_DESCRIPTION6"     //asynParamOctet
#define NDPluginCVOutput7DescriptionString          "NDCV_OUT_DESCRIPTION7"     //asynParamOctet
#define NDPluginCVOutput8DescriptionString          "NDCV_OUT_DESCRIPTION8"     //asynParamOctet
#define NDPluginCVOutput9DescriptionString          "NDCV_OUT_DESCRIPTION9"     //asynParamOctet
#define NDPluginCVOutput10DescriptionString         "NDCV_OUT_DESCRIPTION10"    //asynParamOctet

// File Saving PVs - Currently Unused
// #define NDPluginCVWriteFileString                   "NDCV_FILE"                 //asynParamInt32
// #define NDPluginCVFilenameString                    "NDCV_FILENAME"             //asynParamOctet

// Other records
#define NDPluginCVFunctionDescriptionString         "NDCV_FUN_DESCRIPTION"      //asynParamOctet
#define NDPluginCVStatusMessageString               "NDCV_STATUS"               //asynParamOctet 


/**
 * Enum that maps the openCV data types. Used for copying from NDArray to Mat and back 
 * NOTE: OpenCV does not support an unsigned Int 32 image format 
*/
typedef enum {
    ADCV_Mono_U8            = CV_8UC1,          // Unsigned 8 bit mono
    ADCV_Mono_S8            = CV_8SC1,          // Signed 8 bit mono
    ADCV_RGB_U8             = CV_8UC3,          // Unsigned 8 bit rgb
    ADCV_RGB_S8             = CV_8SC3,          // Signed 8 bit rgb
    ADCV_Mono_U16           = CV_16UC1,         // Unsigned 16 bit mono
    ADCV_Mono_S16           = CV_16SC1,         // Signed 16 bit mono
    ADCV_RGB_U16            = CV_16UC3,         // Unsigned 16 bit rgb
    ADCV_RGB_S16            = CV_16SC3,         // Signed 16 bit rgb
    ADCV_Mono_S32           = CV_32SC1,         // Signed 32 bit mono
    ADCV_RGB_S32            = CV_32SC3,         // Signed 32 bit rgb
    ADCV_Mono_F32           = CV_32FC1,         // Float 32 mono
    ADCV_RGB_F32            = CV_32FC3,         // Float 32 rgb
    ADCV_Mono_F64           = CV_64FC1,         // Float 64 mono
    ADCV_RGB_F64            = CV_64FC3,         // Float 64 rgb
    ADCV_UnsupportedFormat  = -1,
} ADCVFrameFormat_t;


/**
 * Typedef for openCV depth of Mat return.
 * Cant depth + color mode directly
 */
typedef enum {
    ADCV_U8            = CV_8U,          // Unsigned 8
    ADCV_S8            = CV_8S,          // Signed 8
    ADCV_U16           = CV_16U,         // Unsigned 16 bit
    ADCV_S16           = CV_16S,         // Signed 16 bit
    ADCV_S32           = CV_32S,         // Signed 32 bit
    ADCV_F32           = CV_32F,         // Float 32
    ADCV_F64           = CV_64F,         // Float 64
    ADCV_UnsupportedData = -1,
} ADCVDataFormat_t;

/**
 * Color mode of mat, gotten with channels() function
 */
typedef enum {
    ADCV_Mono       = 1,
    ADCV_RGB        = 3,
    ADCV_UnsupportedColor,
} ADCVColorFormat_t;


/* NDPluginCV class that extends base NDPluginDriver class */

class NDPluginCV : public NDPluginDriver{

    public:

        // Constructor/Destructor declarations
        NDPluginCV(const char *portName, int queueSize, int blockingCallbacks,
			const char* NDArrayPort, int NDArrayAddr, int maxBuffers,
            size_t maxMemory, int priority, int stackSize);

        ~NDPluginCV();

        // Process callbacks that will accept arrays from driver
        void processCallbacks(NDArray* pArray);

        //virtual functions that overwrite PluginDriver functions
        virtual asynStatus writeInt32(asynUser* pasynUser, epicsInt32 value);
        //virtual asynStatus writeFloat64(asynUser* pasynUser, epicsFloat64 value);

        // Data type conversion functions (in public because I am working on unit tests)
        ADCVFrameFormat_t getCurrentImageFormat(NDDataType_t dataType, NDColorMode_t colorMode);
        asynStatus getDataTypeFromMat(ADCVDataFormat_t matFormat, NDDataType_t* pdataType);
        asynStatus getColorModeFromMat(ADCVColorFormat_t matFormat, NDColorMode_t* pcolorMode);


    protected:

        //database values for function selectors
        int NDPluginCVFunction1;
        #define NDCV_FIRST_PARAM NDPluginCVFunction1
        int NDPluginCVFunction2;
        int NDPluginCVFunction3;

        // database values for inputs
        int NDPluginCVInput1;
        int NDPluginCVInput2;
        int NDPluginCVInput3;
        int NDPluginCVInput4;
        int NDPluginCVInput5;
        int NDPluginCVInput6;
        int NDPluginCVInput7;
        int NDPluginCVInput8;
        int NDPluginCVInput9;
        int NDPluginCVInput10;

        // database values for inputs
        int NDPluginCVInput1Description;
        int NDPluginCVInput2Description;
        int NDPluginCVInput3Description;
        int NDPluginCVInput4Description;
        int NDPluginCVInput5Description;
        int NDPluginCVInput6Description;
        int NDPluginCVInput7Description;
        int NDPluginCVInput8Description;
        int NDPluginCVInput9Description;
        int NDPluginCVInput10Description;

        // database values for outputs
        int NDPluginCVOutput1;
        int NDPluginCVOutput2;
        int NDPluginCVOutput3;
        int NDPluginCVOutput4;
        int NDPluginCVOutput5;
        int NDPluginCVOutput6;
        int NDPluginCVOutput7;
        int NDPluginCVOutput8;
        int NDPluginCVOutput9;
        int NDPluginCVOutput10;

        // database values for outputs
        int NDPluginCVOutput1Description;
        int NDPluginCVOutput2Description;
        int NDPluginCVOutput3Description;
        int NDPluginCVOutput4Description;
        int NDPluginCVOutput5Description;
        int NDPluginCVOutput6Description;
        int NDPluginCVOutput7Description;
        int NDPluginCVOutput8Description;
        int NDPluginCVOutput9Description;
        int NDPluginCVOutput10Description;

        // File writing db vals - Currently Unused
        // int NDPluginCVWriteFile;
        // int NDPluginCVFilename;

        // Other db values
        int NDPluginCVFunctionDescription;
        int NDPluginCVStatusMessage;
        #define NDCV_LAST_PARAM NDPluginCVStatusMessage

        // Helper library object. Will be created at constructor invocation
	    NDPluginCVHelper* cvHelper;

    private:

        // arrays that will make it easier for iterating over the inputs and outputs
        int inputPVs[NUM_INPUTS];
        int outputPVs[NUM_OUTPUTS];

        int inputDescPVs[NUM_INPUTS];
        int outputDescPVs[NUM_OUTPUTS];

        //functions that assign the PV indexes to arrays
        void assignInputs();
        void assignOutputs();
        void assignInputDescriptions();
        void assignOutputDescriptions();

        // gets function from PV values
        ADCVFunction_t get_function_from_pv(int pvValue, int functionSet);


        // Conversion functions
        asynStatus ndArray2Mat(NDArray* pArray, Mat &pMat, NDDataType_t dataType, NDColorMode_t colorMode);
        asynStatus mat2NDArray(Mat &pMat, NDDataType_t dataType, NDColorMode_t colorMode);

        // function that gets input parameters
        asynStatus getRequiredParams(double* inputs);

        // function that writes output parameters
        asynStatus setOutputParams(double* outputs);

        // Function that calls the appropriate helper library function
        asynStatus updateFunctionDescriptions(ADCVFunction_t function);
        asynStatus processImage(Mat &inputImg);
        // asynStatus writeImageFile(Mat &inputImg);
        asynStatus updatePluginStatus(string statusMessage);
        
};

#define NUM_CV_PARAMS ((int)(&ADUVC_LAST_PARAM - &ADUVC_FIRST_PARAM + 1))

#endif



