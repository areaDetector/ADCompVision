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

//definitions of parameters
#define NDPluginCVComputerVisionFunctionString      "VISION_FUNCTION"   //asynInt32
#define NDPluginCVThresholdValueString              "THRESHOLD_VAL"     //asynFloat64
#define NDPluginCVThresholdRatioString              "THRESHOLD_RATIO"   //asynFloat64
#define NDPluginCVBlurDegreeString                  "BLUR_DEGREE"       //asynInt32
#define NDPluginCVEdgeMethodString                  "EDGE_METHOD"       //asynInt32
#define NDPluginCVROICornerXString                  "ROI_CORNERX"       //asynInt32
#define NDPluginCVROICornerYString                  "ROI_CORNERY"       //asynInt32
#define NDPluginCVROIWidthString                    "ROI_WIDTH"         //asynInt32
#define NDPluginCVROIHeightString                   "ROI_HEIGHT"        //asynInt32


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

        //database values
        int NDPluginCVComputerVisionFunction;

        int NDPluginCVThresholdValue;

        int NDPluginCVThresholdRatio;

        int NDPluginCVBlurDegree;

        int NDPluginCVEdgeMethod;

        int NDPluginCVROICornerX;

        int NDPluginCVROICornerY;

        int NDPluginCVROIWidth;
        
        int NDPluginCVROIHeight;

	    NDPluginCVHelper* cvHelper;

    private:

        //function definitions


        Mat getMatFromNDArray(NDArray* pScratch, NDArray* pArray, int numCols, int numRows);
        void processImage(int visionMode, Mat &img);
        
        //wrapper functions
        Mat canny_wrapper(Mat &img);
        Mat laplacian_wrapper(Mat &img);
        Mat centroid_wrapper(Mat &img);
};

#endif



