/*
 * Header file for helper functions/structs for NDPluginCV
 *
 * Author: Jakub Wlodek
 *
 * Created: June 2018
 *
 */

#ifndef NDPluginCVHelper_H
#define NDPluginCVHelper_H

//include some standard libraries
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <stdio.h>

//OpenCV is used for image manipulation
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

// Num inputs/outputs
#define NUM_INPUTS                  10
#define NUM_OUTPUTS                 10


// Some basic flag types
typedef enum {
    ADCV_NoFunction         = 0,
    ADCV_EdgeDetectionCanny = 1,
    ADCV_Threshold          = 2,
} ADCVFunction_t;

typedef enum {
    cvHelperSuccess         = 0,
    cvHelperError           = -1,
} ADCVStatus_t;

class NDPluginCVHelper {

    public:

        NDPluginCVHelper();
        ~NDPluginCVHelper();

        //function that will print information about an opencv error
        void print_cv_error(Exception &e, const char* functionName);

        ADCVStatus_t canny_edge_detection(Mat &img, double* inputs, double* outputs);

        ADCVStatus_t laplacian_edge_detection(Mat &img, double* inputs, double* outputs);

        ADCVStatus_t threshold_image(Mat &img, double* inputs, double* outputs);

        ADCVStatus_t find_centroids(Mat &img, double* inputs, double* outputs);

        ADCVStatus_t processImage(Mat &image, ADCVFunction_t function, double* inputs, double* outputs);

        //function that will find contours and the centroids of them in a given ROI (NOTE: DEPRACATED)
        Mat centroid_finder(Mat& img, int roiX, int roiY, int roiWidth, int roiHeight, int blurDegree, int threshVal);

    protected:

    private:

};
#endif
