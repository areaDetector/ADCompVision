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

/* 
 * Number of functions in each set of CV functions
 * These numbers are used for finding offsets to find I/O descriptions.
 * 
 * For Example, the first non-empty function of set 2 would have value 1 from the PV,
 * but would be function number N_FUNC_1 + PV val
 * The I/O descriptions would be taken from the array using this value
 * 
 */ 
#define N_FUNC_1                    5
#define N_FUNC_2                    2
#define N_FUNC_3                    1

// Total Number of CV functions
#define NUM_FUNCTIONS               N_FUNC_1 + N_FUNC_2 + N_FUNC_3

// Some basic flag types
typedef enum {
    ADCV_NoFunction         = 0,
    ADCV_EdgeDetectionCanny = 1,
    ADCV_Threshold          = 2,
    ADCV_GaussianBlur       = 3,
    ADCV_Laplacian          = 4,
    ADCV_CentroidFinder     = 5,
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

        string get_input_description(int pvValue, int functionSet);

        string get_output_description(int pvValue, int functionSet);

        ADCVFunction_t get_function_from_pv(int pvValue, int functionSet);

        ADCVStatus_t canny_edge_detection(Mat &img, double* inputs, double* outputs);

        ADCVStatus_t laplacian_edge_detection(Mat &img, double* inputs, double* outputs);

        ADCVStatus_t threshold_image(Mat &img, double* inputs, double* outputs);

        ADCVStatus_t find_centroids(Mat &img, double* inputs, double* outputs);

        ADCVStatus_t gaussian_blur(Mat &img, double* inputs, double* outputs);

        ADCVStatus_t processImage(Mat &image, ADCVFunction_t function, double* inputs, double* outputs);

        //function that will find contours and the centroids of them in a given ROI (NOTE: DEPRACATED)
        Mat centroid_finder(Mat& img, int roiX, int roiY, int roiWidth, int roiHeight, int blurDegree, int threshVal);

    protected:

    private:

};
#endif
