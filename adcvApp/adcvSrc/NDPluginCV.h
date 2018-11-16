/*
 * Main header file for NDPluginCV
 *
 * Includes version number definition, the class itself, function definitions,
 * and parameter definitions. The parameters themselves are initialized
 * in the constructor in the corresponding .cpp file
 *
 * Author: Jakub Wlodek
 *
 * Created: June 2018
 *
 */

//include guard to avoid multiple inclusions
#ifndef NDPluginCV_H
#define NDPluginCV_H

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

//base driver header file
#include "NDPluginDriver.h"
#include "NDPluginCVHelper.h"

//version numbers
#define NDPluginCV_VERSION      0
#define NDPluginCV_REVISION     0
#define NDPluginCV_MODIFICATION 3

/* definitions of parameters */

// CV functions (3)
#define NDPluginCVFunction1String               "NDCV_FUNCTION1"    //asynInt32
#define NDPluginCVFunction2String               "NDCV_FUNCTION2"    //asynInt32
#define NDPluginCVFunction3String               "NDCV_FUNCTION3"    //asynInt32

// Float inputs (5)
#define NDPluginCVFloatInput1String             "NDCV_FLOAT_IN1"    //asynFloat64
#define NDPluginCVFloatInput2String             "NDCV_FLOAT_IN2"    //asynFloat64
#define NDPluginCVFloatInput3String             "NDCV_FLOAT_IN3"    //asynFloat64
#define NDPluginCVFloatInput4String             "NDCV_FLOAT_IN4"    //asynFloat64
#define NDPluginCVFloatInput5String             "NDCV_FLOAT_IN5"    //asynFloat64

// Integer inputs (5)
#define NDPluginCVIntegerInput1String           "NDCV_INT_IN1"      //asynInt32
#define NDPluginCVIntegerInput2String           "NDCV_INT_IN2"      //asynInt32
#define NDPluginCVIntegerInput3String           "NDCV_INT_IN3"      //asynInt32
#define NDPluginCVIntegerInput4String           "NDCV_INT_IN4"      //asynInt32
#define NDPluginCVIntegerInput5String           "NDCV_INT_IN5"      //asynInt32

// Float outputs (3)
#define NDPluginCVFloatOutput1String            "NDCV_FLOAT_OUT1"   //asynFloat64
#define NDPluginCVFloatOutput2String            "NDCV_FLOAT_OUT2"   //asynFloat64
#define NDPluginCVFloatOutput3String            "NDCV_FLOAT_OUT3"   //asynFloat64

// Integer outputs (3)
#define NDPluginCVIntegerOutput1String          "NDCV_INT_OUT1"     //asynInt32
#define NDPluginCVIntegerOutput2String          "NDCV_INT_OUT2"     //asynInt32
#define NDPluginCVIntegerOutput3String          "NDCV_INT_OUT3"     //asynInt32

// Other records
#define NDPluginCVOutputDescriptionString       "NDCV_DESCRIPTION"  //asynOctet


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
    ADCV_RGB_S16            = CV_16SC3,         // Signed 8 bit rgb
    ADCV_Mono_S32           = CV_32SC1,         // Signed 32 bit mono
    ADCV_RGB_S32            = CV_32SC3,         // Signed 32 bit rgb
    ADCV_Mono_F32           = CV_32FC1,         // Float 32 mono
    ADCV_RGB_F32            = CV_32FC3,         // Float 32 rgb
    ADCV_Mono_F64           = CV_64FC1,         // Float 64 mono
    ADCV_RGB_F64            = CV_64FC3,         // Float 64 rgb
    ADCV_UnsupportedFormat  = -1,
} ADCVFrameFormat_t;


//NDPluginCV class that extends base NDPluginDriver class

class NDPluginCV : public NDPluginDriver{

    public:

        NDPluginCV(const char *portName, int queueSize, int blockingCallbacks,
			const char* NDArrayPort, int NDArrayAddr, int maxBuffers,
            size_t maxMemory, int priority, int stackSize);

        ~NDPluginCV();

        void processCallbacks(NDArray* pArray);

        // Data type conversion functions
        ADCVFrameFormat_t getCurrentImageFormat(NDDataType_t dataType, NDColorMode_t colorMode);
        asynStatus getDataTypeFromMat(ADCVFrameFormat_t matFormat, NDDataType_t* pdataType);
        asynStatus getColorModeFromMat(ADCVFrameFormat_t matFormat, NDColorMode_t* pcolorMode);

    protected:

        //database values for function selectors
        int NDPluginCVFunction1;
        int NDPluginCVFunction2;
        int NDPluginCVFunction3;

        // database values for float inputs
        int NDPluginCVFloatInput1;
        int NDPluginCVFloatInput2;
        int NDPluginCVFloatInput3;
        int NDPluginCVFloatInput4;
        int NDPluginCVFloatInput5;

        //database values for integer inputs
        int NDPluginCVIntegerInput1;
        int NDPluginCVIntegerInput2;
        int NDPluginCVIntegerInput3;
        int NDPluginCVIntegerInput4;
        int NDPluginCVIntegerInput5;

        // database values for float outputs
        int NDPluginCVFloatOutput1;
        int NDPluginCVFloatOutput2;
        int NDPluginCVFloatOutput3;
        
        // database values for integer outputs
        int NDPluginCVIntegerOutput1;
        int NDPluginCVIntegerOutput2;
        int NDPluginCVIntegerOutput3;

        // Other db values
        int NDPluginCVOutputDescription;

	    NDPluginCVHelper* cvHelper;

    private:

        //function definitions
        asynStatus ndArray2Mat(NDArray* pArray, Mat* pMat, NDDataType_t dataType, NDColorMode_t colorMode);
        asynStatus mat2NDArray(NDArray* pScratch, Mat* pMat);

        asynStatus processImage(int visionMode, Mat* inputImg, Mat* outputImg);
        
};

#endif



