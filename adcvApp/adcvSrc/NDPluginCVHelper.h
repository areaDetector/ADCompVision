/**
 * NDPluginCVHelper.h
 * 
 * Header file for helper functions/structs for NDPluginCV
 * 
 * This file includes counts for the number of functions in each set, the number of inputs and outputs,
 * various type definitions used by the plugin, and function definitions used by the helper file
 *
 * Author: Jakub Wlodek
 *
 * Created: 26-Jun-2018
 * Last Updated: 14-Jan-2019
 * Copyright (c): Brookhaven National Laboratory 2018-2019
 *
 */

// Include guard
#ifndef NDPluginCVHelper_H
#define NDPluginCVHelper_H

// include some standard libraries
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <stdio.h>

//OpenCV is used for image manipulation
#include <opencv2/opencv.hpp>

#ifdef NDCV_WITH_VIDEO
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>
#endif

// use standard and opencv namespaces
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
#define N_FUNC_1                    6
#define N_FUNC_2                    4
#define N_FUNC_3                    5



// Total Number of CV functions
#define NUM_FUNCTIONS               N_FUNC_1 + N_FUNC_2 + N_FUNC_3


// enum that lists the possible functions supported by ADCompVision. NOTE that they must be in the same order as they are in the DB.
typedef enum {
    ADCV_NoFunction         = 0,
    ADCV_GaussianBlur       = 1,
    ADCV_Threshold          = 2,
    ADCV_Subtract           = 3,
    ADCV_Laplacian          = 4,
    ADCV_EdgeDetectionCanny = 5,
    ADCV_CentroidFinder     = 6,
    ADCV_Sharpen            = 7,
    ADCV_ConvertFormat      = 8,
    ADCV_LogScaling         = 9,
    ADCV_UserDefined        = 10,
    ADCV_ImageStats         = 11,
    ADCV_DistanceCheck      = 12,
    ADCV_VideoRecord        = 13,
} ADCVFunction_t;


//enum that stores file save format
/*
typedef enum {
    ADCV_FileDisable        = 0,
    ADCV_FileJPEG           = 1,
    ADCV_FilePNG            = 2,
    ADCV_FileTIF            = 3,
} ADCVFileFormat_t;
*/


//enum that stores camera depth (8, 10, 12, 14, 16 bit)
typedef enum {
    ADCV_8Bit               = 0,
    ADVC_10Bit              = 1,
    ADCV_12Bit              = 2,
    ADCV_14Bit              = 3,
    ADCV_16Bit              = 4,
} ADCVCameraDepth_t;


// Simple binary status enum
typedef enum {
    cvHelperSuccess         = 0,
    cvHelperError           = -1,
    cvHelperWait            = -2,
} ADCVStatus_t;


/* Helper class that contains CV Wrapper function implemenations */
class NDPluginCVHelper {

    public:

        // variables that can be accessed by the plugin
        string cvHelperStatus;

        // constructor/destructor
        NDPluginCVHelper();
        ~NDPluginCVHelper();

        //function that will print information about an opencv error
        void print_cv_error(Exception &e, const char* functionName);

        //function that converts from rgb to bgr for image passthrough
        ADCVStatus_t fix_coloration(Mat &img);

        // Function that downscales image into 8 bit for functions that require it
        ADCVStatus_t downscale_image_8bit(Mat &img, ADCVCameraDepth_t camera_depth);

        // Additional processing functions (used to clean up wrapper functions)
        double compute_rect_distance(Rect r1, Rect r2);

        void update_str_in(string* new_filepath);
 

        // OpenCV Wrapper functions
        ADCVStatus_t canny_edge_detection(Mat &img, double* inputs, double* outputs);
        ADCVStatus_t laplacian_edge_detection(Mat &img, double* inputs, double* outputs);
        ADCVStatus_t threshold_image(Mat &img, double* inputs, double* outputs);
        ADCVStatus_t find_centroids(Mat &img, double* inputs, double* outputs);
        ADCVStatus_t gaussian_blur(Mat &img, double* inputs, double* outputs);
        ADCVStatus_t subtract_consecutive_images(Mat &img, double* inputs, double* outputs);
        ADCVStatus_t compute_image_stats(Mat &img, double* inputs, double* outputs);
        ADCVStatus_t sharpen_images(Mat &img, double* inputs, double* outputs);
        ADCVStatus_t distance_between_ctrs(Mat &img, double* inputs, double* outputs);
        ADCVStatus_t convert_image_format(Mat &img, double* inputs, double* outputs);
        ADCVStatus_t video_record(Mat &img, double* inputs, double* outputs);
        ADCVStatus_t log_scaling(Mat &img, double* inputs, double* outputs);

        // under development
        ADCVStatus_t movement_vectors(Mat &img, double* inputs, double* outputs);
        ADCVStatus_t obj_identification(Mat &img, double* inputs, double* outputs);


        // IO description helper functions
        void populate_remaining_descriptions(string* inputDesc, string* outputDesc, int nIn, int nOut);
        ADCVStatus_t get_default_description(string* inputDesc, string* outputDesc, string* description);


        // Wrapper function IO descriptions
        ADCVStatus_t get_threshold_description(string* inputDesc, string* outputDesc, string* description);
        ADCVStatus_t get_gaussian_blur_description(string* inputDesc, string* outputDesc, string* description);
        ADCVStatus_t get_laplacian_description(string* inputDesc, string* outputDesc, string* description);
        ADCVStatus_t get_canny_edge_description(string* inputDesc, string* outputDesc, string* description);
        ADCVStatus_t get_centroid_finder_description(string* inputDesc, string* outputDesc, string* description);
        ADCVStatus_t get_subtract_description(string* inputDesc, string* outputDesc, string* description);
        ADCVStatus_t get_image_stats_description(string* inputDesc, string* outputDesc, string* description);
        ADCVStatus_t get_sharpen_description(string* inputDesc, string* outputDesc, string* description);
        ADCVStatus_t get_dist_between_description(string* inputDesc, string* outputDesc, string* description);
        ADCVStatus_t get_convert_format_descripton(string* inputDesc, string* outputDesc, string* description);
        ADCVStatus_t get_video_record_description(string* inputDesc, string* outputDesc, string* description);
        ADCVStatus_t get_log_scaling_description(string* inputDesc, string* outputDesc, string* description);
 
        // Under development
        ADCVStatus_t get_movement_vectors_description(string* inputDesc, string* outputDesc, string* description);
        ADCVStatus_t get_obj_identification_description(string* inputDesc, string* outputDesc, string* description);
        
        // User defined function. Implement these functions in NDPluginCVHelper.cpp to be able to use them within the plugin
        ADCVStatus_t get_user_function_description(string* inputDesc, string* outputDesc, string* description);
        ADCVStatus_t user_function(Mat& img, double* inputs, double* outputs);

        // Functions called from the Plugin itself
        ADCVStatus_t getFunctionDescription(ADCVFunction_t function, string* inputDesc, string* outputDesc, string* description);
        ADCVStatus_t processImage(Mat &image, ADCVFunction_t function, ADCVCameraDepth_t camera_depth, double* inputs, double* outputs);

    protected:

    private:

        // Variables if function requires some information to be stored for multiple function calls

        // general filepath for use with video recording and image saving.
        string filepath = "";


        // movement vector variables (Currently Unused/untested)
        int frameCounter = 0;

        // Temp Mat used by image subtraction. Subtracts one mat from next one
        bool wasComputed = false;
        Mat temporaryImg;

        // variables used for video recording
        #ifdef NDCV_WITH_VIDEO
        VideoWriter video;
        bool isRecording = false;
        #endif

};
#endif
