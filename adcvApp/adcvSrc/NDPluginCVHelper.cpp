/*
 * Helper file for ADCompVision Plugin.
 * This file will contatin all of the OpenCV wrapper functions.
 * The main plugin will call a function that switches on a PV val,
 * and based on the results, passes the image to the correct helper
 * function.
 *
 * Author: Jakub Wlodek
 * Date: June 2018
 *
 */

//include some standard libraries
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <stdio.h>

#include "NDPluginCVHelper.h"

//OpenCV is used for image manipulation
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

const char* libraryName = "NDPluginCVHelper";

/**
 * Simple function that prints OpenCV error information.
 * Used in try/catch blocks
 *
 * @params: e -> exception thrown by OpenCV function
 */
void NDPluginCVHelper::print_cv_error(Exception &e, const char* functionName){
    //cout << "OpenCV error: " << e.err << " code: " << e.code << " file: " << e.file << endl;
    printf("OpenCV Error in function %s: %s code: %d file: %s\n", functionName, e.err, e.code, e.file);
}

/*
#############################################################################
#                                                                           #
# OpenCV wrapper functions. All of these functions will take a Mat* and     #
# pointers for inputs and outputs. Next, it will collect the necessary      #
# inputs, use the correct openCV function on the Mat image, and place any   #
# required values in the outputs array. Finally, it returns a status.       #
#                                                                           #
# The comments before each function describe the input and output values    #
# that the function will create, along with their types                     #
#                                                                           #
#############################################################################
*/


/**
 * Function for canny-based edge detection
 * 
 * @inCount      -> 3
 * @inFormat     -> [Threshold value (Int), Threshold ratio (Int), Blur degree (Int)]
 * 
 * @outCount     -> TODO
 * @outFormat    -> TODO
 */
ADCVStatus_t NDPluginCVHelper::canny_edge_detection(Mat* img, double* inputs, double* outputs){
    const char* functionName = "canny_edge_detection";
    ADCVStatus_t status = cvHelperSuccess;
    int threshVal = inputs[0];
    int threshRatio = inputs[1];
    int blurDegree = inputs[2];
    try{
        blur(*img, *img, Size(blurDegree, blurDegree));
    }catch(Exception &e){
        print_cv_error(e, functionName);
        return cvHelperError;
    }
    try{
        Canny(*img, *img, threshVal, (threshVal*threshRatio));
    }catch(Exception &e){
        print_cv_error(e, functionName);
        return cvHelperError;
    }
    return status;
}

/**
 * Function for canny-based edge detection
 * 
 * @inCount      -> 1
 * @inFormat     -> [Blur degree (Int)]
 * 
 * @outCount     -> TODO
 * @outFormat    -> TODO
 */
ADCVStatus_t NDPluginCVHelper::laplacian_edge_detection(Mat* img, double* inputs, double* outputs){
    const char* functionName = "laplacian_edge_detection";
    int blurDegree = inputs[0];
    ADCVStatus_t status = cvHelperSuccess;
    try{
        GaussianBlur(*img, *img, Size(blurDegree, blurDegree),1, 0, BORDER_DEFAULT);
    }catch(Exception &e){
        print_cv_error(e, functionName);
        return cvHelperError;
    }
    try{
        int depth = img->depth();
        Laplacian(*img, *img, depth);
        convertScaleAbs(*img, *img);
    }catch(Exception &e){
        print_cv_error(e, functionName);
        return cvHelperError;
    }
    return status;
}


/**
 * Function that thresholds an image based on a certain pixel value
 * 
 * @inCount     -> 3
 * @inFormat    -> [Threshhold Value (Int), Max Pixel Value (Int), Threshold Type (Int)]
 * 
 * @outCount    -> TODO
 * @outFormat   -> TODO
 */
ADCVStatus_t NDPluginCVHelper::threshold_image(Mat* img, double* inputs, double* outputs){
    const char* functionName = "threshold_image";
    ADCVStatus_t status = cvHelperSuccess;
    int threshVal = inputs[0];
    int threshMax = inputs[1];
    int threshType = inputs[2];
    try{
        threshold(*img, *img, threshVal, threshMax, threshType);
    }catch(Exception &e){
        status = cvHelperError;
        print_cv_error(e, functionName);
    }
    return status;
}


ADCVStatus_t NDPluginCVHelper::find_centroids(Mat* img, double* inputs, double* outputs){
    static const char* functionName = "find_centroids";
    ADCVStatus_t status = cvHelperSuccess;
    int blurDegree = inputs[0];
    int thresholdVal = inputs[1];
    try{
        GaussianBlur(*img, *img, Size(blurDegree, blurDegree), 0);
        threshold(*img, *img, thresholdVal, 255, THRESH_BINARY);
        vector<vector<Point>> contours;
        vector<Vec4i> heirarchy;

        findContours(*img, contours, heirarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0,0));
        vector<Moments> contour_moments(contours.size());
        int i, j, k;
        for(i = 0; i < contours.size(); i++){
            contour_moments[i] = moments(contours[i], false);
        }
        vector<Point2f> contour_centroids(contours.size());
        for(j = 0; j < contours.size(); j++){
            contour_centroids[i] = Point2f((contour_moments[j].m10/contour_moments[j].m00), (contour_moments[j].m01/contour_moments[j].m00));
        }
        for(k = 0; k < contour_centroids.size(); k++){
            if(k%2==0){
                outputs[k] == contour_centroids[k/2].x;
            }
            else{
                outputs[k] == contour_centroids[k/2].y;
            }

            if(k == NUM_OUTPUTS) break;
        }

    } catch(Exception &e){
        status = cvHelperError;
        print_cv_error(e, functionName);
    }
    return status;
}

//------------------------ End of OpenCV wrapper functions -------------------------------------------------


/*

    TODO: Rewrite this function/test chainging core functions to achieve the same result

 * Function that finds centroid of all contours in a given region of interest.
 * The following process is taken:
 * Gaussian blur -> threshold -> crop -> find contours+moments -> find centroids
 * 
 * @params: img -> image to be processed
 * @params: roiX -> x-coordinate of upper left corner of ROI (region of interest)
 * @params: roiY -> y-coordinate of upper left corner of ROI
 * @params: roiWidth -> width of the ROI
 * @params: roiHeight -> height of the ROI. Note that the height in an image is measured from the top down
 * @params: blurDegree -> size of kernel in Gaussian blur
 * @params: threshVal -> cutoff  for threshold
 * @return: cropped image with detected contours and centroids
Mat NDPluginCVHelper::centroid_finder(Mat &img, int roiX, int roiY, int roiWidth, int roiHeight, int blurDegree, int threshVal){
    Mat afterBlur, afterThresh, afterCrop, cropOriginal;
    GaussianBlur(img, afterBlur, Size(blurDegree,blurDegree), 0);
    threshold(afterBlur, afterThresh, threshVal, 255, THRESH_BINARY);
    Rect myROI(roiX, roiY, roiWidth, roiHeight);
    afterCrop = afterThresh(myROI);
    cropOriginal = img(myROI);
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    findContours(afterCrop, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0,0));
    vector<Moments> contour_moments(contours.size());
    for(int i = 0; i< contours.size(); i++){
        contour_moments[i] = moments(contours[i], false);
    }
    vector<Point2f> contour_centroids(contours.size());
    for(int i = 0; i< contours.size(); i++){
        contour_centroids[i] = Point2f((contour_moments[i].m10/contour_moments[i].m00), (contour_moments[i].m01/contour_moments[i].m00));
    }
    for(int i = 0; i< contours.size(); i++){
        drawContours(cropOriginal, contours, i, Scalar(0,255,0), 2, 8, hierarchy, 0, Point());
        circle(cropOriginal, contour_centroids[i], 3, Scalar(255,0,0), -1, 8, 0);
    }
    imshow("contours+centroids", cropOriginal);
    waitKey(0);
    return img;
}

*/


/**
 * Function that is called from the ADCompVision plugin. It detects which function is being requested, and calls the appropriate
 * opencv wrapper function from those above.
 * 
 * @params: image       -> pointer to Mat object
 * @params: function    -> type of CV function to perform
 * @params: inputs      -> array with inputs for functions
 * @params: outputs     -> array for outputs of functions
 * @return: status      -> check if library function completed successfully
 */
ADCVStatus_t NDPluginCVHelper::processImage(Mat* image, ADCVFunction_t function, double* inputs, double* outputs){
    const char* functionName = "processImage";
    ADCVStatus_t status;

    switch(function){
        case ADCV_EdgeDetectionCanny:
            status = canny_edge_detection(image, inputs, outputs);
            break;
        case ADCV_Threshold:
            status = threshold_image(image, inputs, outputs);
            break;
        default:
            status = cvHelperError;
            break;
    }

    if(status == cvHelperError){
        printf("%s::%s Error in helper library\n", libraryName, functionName);
    }
    return status;
}


/* Basic constructor/destructor, used by plugin to call processImage */

NDPluginCVHelper::NDPluginCVHelper(){ }

NDPluginCVHelper::~NDPluginCVHelper(){ delete this; }
