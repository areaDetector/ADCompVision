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

//version numbers 
#define NDPluginCV_VERSION      0
#define NDPluginCV_REVISION     0
#define NDPluginCV_MODIFICATION 0

//definitions of parameters
#define NDPluginCVComputerVisionFunctionString "VISION_FUNCTION" //asynInt32
#define NDPluginCVThresholdValueString "THRESHOLD_VAL" //asynFloat64
#define NDPluginCVThresholdRatioString "THRESHOLD_RATIO" //asynFloat64
#define NDPluginCVBlurDegreeString "BLUR_DEGREE" //asynInt32
#define NDPluginCVEdgeMethodString "EDGE_METHOD" //asynInt32
#define NDPluginCVROICornerXString "ROI_CORNERX" //asynInt32
#define NDPluginCVROIDCornerYString "ROI_CORNERY" //asynInt32
#define NDPluginCVROIWidthString "ROI_WIDTH" //asynInt32
#define NDPluginCVROIHeightString "ROI_HEIGHT" //asynInt32


//NDPluginCV class that extends base NDPluginDriver class

class NDPluginCV : public NDPluginDriver{

    public:

        NDPluginCV(const char *portName, int queueSize, int blockingCallbacks,
			const char* NDArrayPort, int NDArrayAddr, int maxBuffers,
            size_t maxMemory, int priority, int stackSize);
        
        void processCallbacks(NDArray* pArray);

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

    private:

        //function definitions
        Mat getMatFromNDArray(NDArray* pScratch, NDArray* pArray, int numCols, int numRows);
        void processImage(int visionMode, Mat &img);
        
        //wrapper functions
        Mat canny_wrapper();
        Mat laplacian_wrapper();
        Mat centroid_wrapper();
};

#endif



