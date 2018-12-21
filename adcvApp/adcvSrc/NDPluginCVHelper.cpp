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
 * Copyright (c): Brookhaven National Laboratory 2018
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
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

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
    printf("OpenCV Error in function %s: %s code: %d file: %s\n", functionName, e.err.c_str(), e.code, e.file.c_str());
}


/**
 * Function that takes PV value from the plugin driver, and converts it into the ADCVFunction_t 
 * enum type. This is used to decide which function the plugin is to perform as well
 * as to compute Input/Output descriptions
 * 
 * @params: pvValue         -> value of the PV when it is changed
 * @params: functionSet     -> the set from which the function set came from. currently (1-3)
 * @return: function        -> returns the function as an ADCVFunction_t enum
 */
ADCVFunction_t NDPluginCVHelper::get_function_from_pv(int pvValue, int functionSet){
    const char* functionName = "get_function_from_pv";
    if(functionSet == 1){
        return (ADCVFunction_t) pvValue;
    }
    if(functionSet == 2){
        return (ADCVFunction_t) (N_FUNC_1 + pvValue - 1);
    }
    if(functionSet == 3){
        return (ADCVFunction_t) (N_FUNC_1 + N_FUNC_2 - 1 + pvValue - 1);
    }
    printf("%s::%s ERROR: Couldn't find correct function val\n", libraryName, functionName);
    return ADCV_NoFunction;
}



/*
#############################################################################
#                                                                           #
# OpenCV wrapper functions. All of these functions will take a Mat and     #
# pointers for inputs and outputs. Next, it will collect the necessary      #
# inputs, use the correct openCV function on the Mat image, and place any   #
# required values in the outputs array. Finally, it returns a status.       #
#                                                                           #
# The comments before each function describe the input and output values    #
# that the function will create, along with their types                     #
#                                                                           #
#############################################################################
*/


//------------- Template for OpenCV function wrapper -------------------

/*
**
 * WRAPPER  ->  YOURFUNCTIONNAME
 * YOUR_FUNCTION_DESCRIPTION
 *
 * @inCount     -> n
 * @inFormat    -> [Param1 (Int), Param2 (Double) ...]
 *
 * @outCount    -> n
 * @outFormat   -> [Param1 (Int), Param2 (Double) ...]
 *
ADCVStatus_t NDPluginCVHelper::YOURFUNCTION(Mat &img, double* inputs, double* outputs){
    const char* functionName = "YOURFUNCTION";
    ADCVStatus_t status = cvHelperSuccess;
    param1 = inputs[0];
    param2 = inputs[1];
    .
    .
    .

    try{
        // Process your image here
        // Don't make copies, pass img, img as input and output to OpenCV.
        // Set output values with output[n] = value. cast non-double values to double
        // If you need more inputs or outputs, add more PVs following previous examples.
    }catch(Exception &e){
        print_cv_error(e, functionName);
        status = cvHelperError;
    }
    return status;
}
*/

//------------- OpenCV function wrapper implementations -------------------


/**
 * WRAPPER      -> Canny Edge Detector
 * Function for canny-based edge detection. First, we ensure that the image is grayscale. Then, the image is blurred, so that only
 * strong edges remain. Then, a threshold is applied to the image in order to further reinforce strong edges. Finally, the canny
 * algorithm is applied to the image, and the edges are displayed. The function outputs some information based on the detected
 * edges that can assist with object detection/identification: the top, bottom, left, and right pixels are the min and max X and
 * Y pixel values that appear on one of the edges. The horizontal and vertical size and center give you the spacing between
 * these min and max values and their midpoint.
 * 
 * @inCount     -> 3
 * @inFormat    -> [Threshold value (Int), Threshold ratio (Int), Blur degree (Int), Kernel Size (Int)]
 *
 * @outCount    -> 8
 * @outFormat   -> [Horizontal Center, Horizontal Size, Vertical Center, Vertical Size, Top Pixel, Bottom Pixel, Left Pixel, Right Pixel]
 */
ADCVStatus_t NDPluginCVHelper::canny_edge_detection(Mat &img, double* inputs, double* outputs){
    const char* functionName = "canny_edge_detection";
    ADCVStatus_t status = cvHelperSuccess;
    int threshVal = inputs[0];
    int threshRatio = inputs[1];
    int blurDegree = inputs[2];
    int kernelSize = inputs[3];
    // If image isn't mono, we need to convert it first
    if(img.channels()!=2){
        cvtColor(img, img, COLOR_BGR2GRAY);
    }
    try{
        blur(img, img, Size(blurDegree, blurDegree));
        Canny(img, img, threshVal, (threshVal*threshRatio), kernelSize);
        // set output params
        int i, j;
        unsigned char* outData = (unsigned char *)img.data;
        int bottomPixel = -1;
        int topPixel = 100000;
        int leftPixel = 100000;
        int rightPixel = -1;
        for(j = 0; j< img.cols; j++){
            for(i = 0; i< img.rows; i++){
                int newPixel = outData[img.cols * i + j];
                if(newPixel != 0){
                    if(j<leftPixel) leftPixel = j;
                    if(j>rightPixel) rightPixel = j;
                    if(i<topPixel) topPixel = i;
                    if(i>bottomPixel) bottomPixel = i;
                }
            }
        }
        topPixel = img.size().height - topPixel;
        bottomPixel = img.size().height - bottomPixel;
        int h_size = rightPixel - leftPixel;
        int v_size = topPixel - bottomPixel;
        int h_center = (h_size/2) + leftPixel;
        int v_center = (v_size/2)+ bottomPixel;
        outputs[0] = h_center;
        outputs[1] = h_size;
        outputs[2] = v_center;
        outputs[3] = v_size;
        outputs[4] = topPixel;
        outputs[5] = bottomPixel;
        outputs[6] = leftPixel;
        outputs[7] = rightPixel;
    }catch(Exception &e){
        print_cv_error(e, functionName);
        return cvHelperError;
    }
    return status;
}


/**
 * WRAPPER      -> Laplacian Edge Detector
 * Function for laplacian-based edge detection. First, the image is converted to grayscale if it is not already.
 * Next, the image is blurred using a gaussian kernel to emphasize edges. Then a laplacian kernel runs over
 * the images assigning a 'sharpness' value to each pixel. The sharpest values are hard edges from black to white.
 * 
 * @inCount     -> 1
 * @inFormat    -> [Blur degree (Int)]
 * 
 * @outCount    -> 0
 * @outFormat   -> None
 */
ADCVStatus_t NDPluginCVHelper::laplacian_edge_detection(Mat &img, double* inputs, double* outputs){
    const char* functionName = "laplacian_edge_detection";
    int blurDegree = inputs[0];
    ADCVStatus_t status = cvHelperSuccess;
    if(img.channels()!=2){
        cvtColor(img, img, COLOR_BGR2GRAY);
    }
    try{
        GaussianBlur(img, img, Size(blurDegree, blurDegree),1, 0, BORDER_DEFAULT);
        int depth = img.depth();
        Laplacian(img, img, depth);
        convertScaleAbs(img, img);
    }catch(Exception &e){
        print_cv_error(e, functionName);
        return cvHelperError;
    }
    return status;
}


/**
 * WRAPPER      -> Threshold Image
 * Function that thresholds an image based on a certain pixel value. First, the image is converted to grayscale.
 * RGB images cannot be thresholded. For each pixel, if the grayscale value is larger than the threshold, set it 
 * to white, otherwise set it to black. Creates a binary image
 * 
 * @inCount     -> 3
 * @inFormat    -> [Threshhold Value (Int), Max Pixel Value (Int)]
 * 
 * @outCount    -> 0
 * @outFormat   -> None
 */
ADCVStatus_t NDPluginCVHelper::threshold_image(Mat &img, double* inputs, double* outputs){
    const char* functionName = "threshold_image";
    ADCVStatus_t status = cvHelperSuccess;
    
    if(img.channels()!=2){
        cvtColor(img, img, COLOR_BGR2GRAY);
    }
    //imwrite("/home/jwlodek/Documents/testGray.jpg", img);
    int threshVal = (int) inputs[0];
    int threshMax = (int) inputs[1];
    //printf("%s::%s Recieving thresh val %d, thresh max %d, image size %d\n", libraryName, functionName, threshVal, threshMax, img.channels());
    try{
        threshold(img, img, threshVal, threshMax, THRESH_BINARY);
        //imwrite("/home/jwlodek/Documents/testThresh.jpg", img);
    }catch(Exception &e){
        status = cvHelperError;
        print_cv_error(e, functionName);
    }
    return status;
}


/**
 * WRAPPER      -> Find Object Centroids
 * Function for finding centroids of objects in an image. Useful for alignment of objects
 * First, blur the object based on a certain blur degree (kernel size). Then threshold the image
 * based on a certain threshold value. Then find contours in the image using the findContours()
 * function. Then get the centroids from the contour objects. Draw the contours and centroids on 
 * the image. Set the first 5 centroid coordinates to the output values. A size filter can also
 * be used to remove contours that are too large, removing contours that span the entire size of
 * the image. Any contour with area > upper threshold is removed, and any lower than lower threshold
 * 
 * @inCount     -> 5
 * @inFormat    -> [Num Largest Contours (Int), Blur Degree (Int), Threshold Value (Int), Upper Size Threshold (Int), Lower Size Threshold (Int)]
 * 
 * @outCount    -> 2-10
 * @outFormat   -> [CentroidX (Int), CentroidY (Int) ... ]
 */
ADCVStatus_t NDPluginCVHelper::find_centroids(Mat &img, double* inputs, double* outputs){
    static const char* functionName = "find_centroids";
    ADCVStatus_t status = cvHelperSuccess;
    size_t numLargestContours = (size_t) inputs[0];
    int blurDegree = (int) inputs[1];
    int thresholdVal = (int) inputs[2];
    double upperSizeThreshold = inputs[3];
    double lowerSizeThreshold = inputs[4];
    try{
        // first we need to convert to grayscale if necessary
        if(img.channels()!=2){
            cvtColor(img, img, COLOR_BGR2GRAY);
        }
        GaussianBlur(img, img, Size(blurDegree, blurDegree), 0);
        threshold(img, img, thresholdVal, 255, THRESH_BINARY);
        vector<vector<Point>> contours;
        vector<Vec4i> heirarchy;

        findContours(img, contours, heirarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0,0));
        if(contours.size() < numLargestContours){
            numLargestContours = contours.size();
        }
        vector<vector<Point>> largestContours(numLargestContours);
        size_t a, b;
        for(a = 0; a< numLargestContours; a++){
            vector<Point> largestContour = contours[0];
            double largestArea = -1;
            size_t pos = 0;
            for(b = 0; b< contours.size(); b++){
                double area = contourArea(contours[b]);
                if(area > largestArea && area < upperSizeThreshold){
                    largestContour = contours[b];
                    largestArea = area;
                    pos = b;
                }
            }
            if(largestArea < lowerSizeThreshold){
                numLargestContours = a;
                break;
            }
            largestContours[a] = largestContour;
            //printf("largest contours is of size %lf out of %d\n", largestArea, 640*480);
            contours.erase(contours.begin() + pos);
        }
        vector<Moments> contour_moments(largestContours.size());
        size_t i, j, k, l;
        for(i = 0; i < largestContours.size(); i++){
            contour_moments[i] = moments(largestContours[i], false);
        }
        vector<Point2f> contour_centroids(contours.size());
        for(j = 0; j < largestContours.size(); j++){
            contour_centroids[j] = Point2f((contour_moments[j].m10/contour_moments[j].m00), (contour_moments[j].m01/contour_moments[j].m00));
        }
        int counter = 0;
        for(k = 0; k < contour_centroids.size(); k++){
            outputs[counter] = contour_centroids[k].x;
            if(contour_centroids[k].y != 0) outputs[counter+1] = img.size().height - contour_centroids[k].y;
            counter = counter + 2;
            if(counter >= NUM_OUTPUTS) break;
        }
        cvtColor(img, img, COLOR_GRAY2BGR);
        for(l = 0; l< largestContours.size(); l++){
            if(l == numLargestContours) break;
            drawContours(img, largestContours, l, Scalar(0, 0, 255), 2, 8, heirarchy, 0, Point());
            circle(img, contour_centroids[l], 3, Scalar(255,0,0), -1, 8, 0);
        }
    } catch(Exception &e){
        status = cvHelperError;
        print_cv_error(e, functionName);
    }
    return status;
}


/**
 * WRAPPER  ->  gaussian_blur
 * Blurs image based on a gaussian kernel. A gaussian kernel is simply a matrix of a set size that
 * fills Gaussian properties.
 *
 * @inCount     -> 1
 * @inFormat    -> [blurDegree (Int)]
 *
 * @outCount    -> 0
 * @outFormat   -> None
 */
ADCVStatus_t NDPluginCVHelper::gaussian_blur(Mat &img, double* inputs, double* outputs){
    const char* functionName = "gaussian_blur";
    ADCVStatus_t status = cvHelperSuccess;
    int blurDegree = inputs[0];
    //imwrite("/home/jwlodek/Documents/testinp.jpg", img);
    try{
        if(img.channels() == 3){
            cvtColor(img, img, COLOR_RGB2BGR);
        }
        GaussianBlur(img, img, Size(blurDegree, blurDegree), 1, 0, BORDER_DEFAULT);
        //imwrite("/home/jwlodek/Documents/testgaussian.jpg", img);
    }catch(Exception &e){
        print_cv_error(e, functionName);
        status = cvHelperError;
    }
    return status;
}


//------------------------ End of OpenCV wrapper functions -------------------------------------------------

/*
#############################################################################
#                                                                           #
# Wrapper function I/O description setters. All of these functions will     #
# take an array of strings for inputs and outputs, and an a single string   #
# for an overall description. These are simply populated, and then a func   #                                                                       
# that fills the remaining spots is called.                                 #
#                                                                           #
#############################################################################
*/


//------------- Template for wrapper I/O description function  -------------------

/*
 *
 * Function that sets the I/O descriptions for YOURFUNCTION
 * 
 * @params[out]: inputDesc      -> array of input descriptions
 * @params[out]: outputDesc     -> array of output descriptions
 * @params[out]: description    -> overall function usage description
 * @return: void
 *
ADCVStatus_t NDPluginCVHelper::get_YOURFUNCTION_description(string* inputDesc, string* outputDesc, string* description){
    ADCVStatus_t status = cvHelperSuccess;
    int numInput = ?;
    int numOutput = ?;
    inputDesc[0] = "Input 1 Description";
    inputDesc[1] = "Input 2 Description";
    .
    .
    .
    outputDesc[0] = "Output 1 Description";
    outputDesc[1] = "Output 2 Description";
    .
    .
    .
    description = "Description of YOURFUNCTION";
    populate_remaining_descriptions(inputDesc, outputDesc, numInput, numOutput);
    return status
}
*/


//------------------------ Wrapper function I/O description implementation -------------------------------


/**
 * Simple function that populates the remaining I/O descriptions with an unused
 * tag
 * 
 * @params[out]: inputDesc      -> array of input descriptions
 * @params[out]: outputDesc     -> array of output descriptions
 * @params[in]:  nIn            -> number of inputs
 * @params[in]:  nOut           -> number of outputs
 * @return: void
 */
void NDPluginCVHelper::populate_remaining_descriptions(string* inputDesc, string* outputDesc, int nIn, int nOut){
    int i, j;
    for(i = nIn; i< NUM_INPUTS; i++){
        inputDesc[i] = "Not Used";
    }
    for(j = nOut; j< NUM_OUTPUTS; j++){
        outputDesc[j] = "Not Used";
    }
}


/**
 * Function that sets the I/O descriptions for thresholding
 * 
 * @params[out]: inputDesc      -> array of input descriptions
 * @params[out]: outputDesc     -> array of output descriptions
 * @params[out]: description    -> overall function usage description
 * @return: void
 */
ADCVStatus_t NDPluginCVHelper::get_threshold_description(string* inputDesc, string* outputDesc, string* description){
    ADCVStatus_t status = cvHelperSuccess;
    int numInput = 2;
    int numOutput = 0;
    inputDesc[0] = "Threshold Value (Int)";
    inputDesc[1] = "Max Pixel Value (Int)";
    *description = "Will create binary image with cutoff at Threshold Val";
    populate_remaining_descriptions(inputDesc, outputDesc, numInput, numOutput);
    return status;
}


/**
 * Function that sets the I/O descriptions for thresholding
 * 
 * @params[out]: inputDesc      -> array of input descriptions
 * @params[out]: outputDesc     -> array of output descriptions
 * @params[out]: description    -> overall function usage description
 * @return: void
 */
ADCVStatus_t NDPluginCVHelper::get_gaussian_blur_description(string* inputDesc, string* outputDesc, string* description){
    ADCVStatus_t status = cvHelperSuccess;
    int numInput = 1;
    int numOutput = 0;
    inputDesc[0] = "Blur Degree (Int)";
    *description = "Will blur image based on certain kernel blur degree (odd number int)";
    populate_remaining_descriptions(inputDesc, outputDesc, numInput, numOutput);
    return status;
}


/**
 * Function that sets the I/O descriptions for Laplacian
 * 
 * @params[out]: inputDesc      -> array of input descriptions
 * @params[out]: outputDesc     -> array of output descriptions
 * @params[out]: description    -> overall function usage description
 * @return: void
 */
ADCVStatus_t NDPluginCVHelper::get_laplacian_description(string* inputDesc, string* outputDesc, string* description){
    ADCVStatus_t status = cvHelperSuccess;
    int numInput = 1;
    int numOutput = 0;
    inputDesc[0] = "Blur Degree (Int)";
    *description = "Edge detection using a combination of a Gaussian Blur kernel and a Laplacian kernel";
    populate_remaining_descriptions(inputDesc, outputDesc, numInput, numOutput);
    return status;
}


/**
 * Function that sets the I/O descriptions for Laplacian
 * 
 * @params[out]: inputDesc      -> array of input descriptions
 * @params[out]: outputDesc     -> array of output descriptions
 * @params[out]: description    -> overall function usage description
 * @return: void
 */
ADCVStatus_t NDPluginCVHelper::get_canny_edge_description(string* inputDesc, string* outputDesc, string* description){
    ADCVStatus_t status = cvHelperSuccess;
    int numInput = 4;
    int numOutput = 8;
    inputDesc[0] = "Threshold Value (Int) Ex. 100";
    inputDesc[1] = "Threshold ratio (Int) Ex. 3";
    inputDesc[2] = "Blur Degree (Int) Ex. 3";
    inputDesc[3] = "Kernel Size (Int) Ex. 3";
    outputDesc[0] = "Horizontal Center";
    outputDesc[1] = "Horizontal Size";
    outputDesc[2] = "Vertical Center";
    outputDesc[3] = "Vertical Size";
    outputDesc[4] = "Top Pixel";
    outputDesc[5] = "Bottom Pixel";
    outputDesc[6] = "Left Pixel";
    outputDesc[7] = "Right Pixel";
    *description = "Edge detection using the 'Canny' function. First blurs the image, then thresholds, then runs the canny algorithm.";
    populate_remaining_descriptions(inputDesc, outputDesc, numInput, numOutput);
    return status;
}


/**
 * Function that sets the I/O descriptions for Centroid identification
 * 
 * @params[out]: inputDesc      -> array of input descriptions
 * @params[out]: outputDesc     -> array of output descriptions
 * @params[out]: description    -> overall function usage description
 * @return: void
 */
ADCVStatus_t NDPluginCVHelper::get_centroid_finder_description(string* inputDesc, string* outputDesc, string* description){
    ADCVStatus_t status = cvHelperSuccess;
    int numInput = 5;
    int numOutput = 10;
    inputDesc[0] = "Num Largest Contours (Int 1 - 5)";
    inputDesc[1] = "Blur degree (Int) Ex. 3";
    inputDesc[2] = "Threshold Value (Int) Ex. 100";
    inputDesc[3] = "Upper Size Threshold Ex. 600*400";
    inputDesc[4] = "Lower Size Threshold Ex. 400";
    outputDesc[0] = "Centroid 1 X";
    outputDesc[1] = "Centroid 1 Y";
    outputDesc[2] = "Centroid 2 X";
    outputDesc[3] = "Centroid 2 Y";
    outputDesc[4] = "Centroid 3 X";
    outputDesc[5] = "Centroid 3 Y";
    outputDesc[6] = "Centroid 4 X";
    outputDesc[7] = "Centroid 4 Y";
    outputDesc[8] = "Centroid 5 X";
    outputDesc[9] = "Centroid 5 Y";
    *description = "Centroid computation. Thresholds, then finds centroid. Thresholds used to remove contours by area";
    populate_remaining_descriptions(inputDesc, outputDesc, numInput, numOutput);
    return status;
}


/**
 * Function that sets default I/O descriptions
 * 
 * @params[out]: inputDesc      -> array of input descriptions
 * @params[out]: outputDesc     -> array of output descriptions
 * @params[out]: description    -> overall function usage description
 * @return: void
 */
ADCVStatus_t NDPluginCVHelper::get_default_description(string* inputDesc, string* outputDesc, string* description){
    ADCVStatus_t status = cvHelperError;
    int i, j;
    for(i = 0; i< NUM_INPUTS; i++){
        inputDesc[i] = "Not Available";
    }
    for(j = 0; j< NUM_OUTPUTS; j++){
        outputDesc[j] = "Not Available";
    }
    *description = "None Available";
    return status;
}

/* ---------------------- Functions called from the EPICS Plugin implementation ----------------------- */


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
ADCVStatus_t NDPluginCVHelper::processImage(Mat &image, ADCVFunction_t function, double* inputs, double* outputs){
    const char* functionName = "processImage";
    ADCVStatus_t status;

    switch(function){
        case ADCV_Threshold:
            status = threshold_image(image, inputs, outputs);
            break;
        case ADCV_GaussianBlur:
            status = gaussian_blur(image, inputs, outputs);
            break;
        case ADCV_EdgeDetectionCanny:
            status = canny_edge_detection(image, inputs, outputs);
            break;
        case ADCV_CentroidFinder:
            status = find_centroids(image, inputs, outputs);
            break;
        case ADCV_Laplacian:
            status = laplacian_edge_detection(image, inputs, outputs);
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


/**
 * This function is called from the ADCompVision plugin. It returns information regarding the input types, output 
 * types, and the function itself, for display in the U.I.
 * 
 * @params[in]:  function       -> function type
 * @params[out]: inputDesc      -> Array of input descriptions
 * @params[out]: outputDesc     -> Array of output descriptions
 * @params[out]: description    -> Description of the function
 * @return: cvHelperSuccess if function desc defined, otherwise cvHelperError
 */
ADCVStatus_t NDPluginCVHelper::getFunctionDescription(ADCVFunction_t function, string* inputDesc, string* outputDesc, string* description){
    const char* functionName = "getFunctionDescription";
    ADCVStatus_t status;

    switch(function){
        case ADCV_Threshold:
            status = get_threshold_description(inputDesc, outputDesc, description);
            break;
        case ADCV_GaussianBlur:
            status = get_gaussian_blur_description(inputDesc, outputDesc, description);
            break;
        case ADCV_Laplacian:
            status = get_laplacian_description(inputDesc, outputDesc, description);
            break;
        case ADCV_EdgeDetectionCanny:
            status = get_canny_edge_description(inputDesc, outputDesc, description);
            break;
        case ADCV_CentroidFinder:
            status = get_centroid_finder_description(inputDesc, outputDesc, description);
            break;
        default:
            status = get_default_description(inputDesc, outputDesc, description);
            break;
    }
    if(status == cvHelperError){
        printf("%s::%s Error, Function does not support I/O descriptions\n", libraryName, functionName);
    }
    return status;

}


/* Basic constructor/destructor, used by plugin to call processImage */

NDPluginCVHelper::NDPluginCVHelper(){ }

NDPluginCVHelper::~NDPluginCVHelper(){ }
