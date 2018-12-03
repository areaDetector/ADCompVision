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

const char* input_descriptions[NUM_FUNCTIONS] = {"No Inputs", "[Threshold value (Int), Threshold ratio (Int), Blur degree (Int)]",
                               "[Threshhold Value (Int), Max Pixel Value (Int), Threshold Type (Int)]", "[Blur degree (Int)]",
                               "[Blur degree (Int)]", "[Blur Degree (Int), Threshold Value (Int)]"};


const char* output_descriptions[NUM_FUNCTIONS] = {"TODO", "TODO", "TODO", "TODO", "TODO", "[CentroidX (Double), CentroidY (Double) ... ]"};

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
        return (ADCVFunction_t) (N_FUNC_1 + pvValue);
    }
    if(functionSet == 3){
        return (ADCVFunction_t) (N_FUNC_1 + N_FUNC_2 - 1 +pvValue);
    }
    printf("%s::%s ERROR: Couldn't find correct function val\n", libraryName, functionName);
    return ADCV_NoFunction;
}



/**
 * Function that gets a description of the input values necessary for using the selected function
 * 
 * @params: pvValue         -> value of the PV when it is changed
 * @params: functionSet     -> the set from which the function set came from. currently (1-3)
 * @return: description     -> description of the inputs
 */
const char* NDPluginCVHelper::get_input_description(int pvValue, int functionSet){
    const char* functionName = "get_input_description";
    ADCVFunction_t function = get_function_from_pv(pvValue, functionSet);
    if(function == ADCV_NoFunction){
        printf("%s::%s No function selected\n", libraryName, functionName);
        return input_descriptions[function];
    }
    else{
        return input_descriptions[function];
    }
}


/**
 * Function that gets a description of the ouput values for the selected function
 * 
 * @params: pvValue         -> value of the PV when it is changed
 * @params: functionSet     -> the set from which the function set came from. currently (1-3)
 * @return: description     -> description of the outputs
 */
const char* NDPluginCVHelper::get_output_description(int pvValue, int functionSet){
    const char* functionName = "get_output_description";
    ADCVFunction_t function = get_function_from_pv(pvValue, functionSet);
    if(function == ADCV_NoFunction){
        printf("%s::%s No function selected\n", libraryName, functionName);
        return output_descriptions[function];
    }
    else{
        return output_descriptions[function];
    }
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
 * Function for canny-based edge detection
 * 
 * @inCount     -> 3
 * @inFormat    -> [Threshold value (Int), Threshold ratio (Int), Blur degree (Int)]
 *
 * @outCount    -> TODO
 * @outFormat   -> TODO
 */
ADCVStatus_t NDPluginCVHelper::canny_edge_detection(Mat &img, double* inputs, double* outputs){
    const char* functionName = "canny_edge_detection";
    ADCVStatus_t status = cvHelperSuccess;
    int threshVal = inputs[0];
    int threshRatio = inputs[1];
    int blurDegree = inputs[2];
    try{
        blur(img, img, Size(blurDegree, blurDegree));
        Canny(img, img, threshVal, (threshVal*threshRatio));
    }catch(Exception &e){
        print_cv_error(e, functionName);
        return cvHelperError;
    }
    return status;
}


/**
 * WRAPPER      -> Laplacian Edge Detector
 * Function for laplacian-based edge detection
 * 
 * @inCount     -> 1
 * @inFormat    -> [Blur degree (Int)]
 * 
 * @outCount    -> TODO
 * @outFormat   -> TODO
 */
ADCVStatus_t NDPluginCVHelper::laplacian_edge_detection(Mat &img, double* inputs, double* outputs){
    const char* functionName = "laplacian_edge_detection";
    int blurDegree = inputs[0];
    ADCVStatus_t status = cvHelperSuccess;
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
 * Function that thresholds an image based on a certain pixel value
 * 
 * @inCount     -> 3
 * @inFormat    -> [Threshhold Value (Int), Max Pixel Value (Int), Threshold Type (Int)]
 * 
 * @outCount    -> TODO
 * @outFormat   -> TODO
 */
ADCVStatus_t NDPluginCVHelper::threshold_image(Mat &img, double* inputs, double* outputs){
    const char* functionName = "threshold_image";
    ADCVStatus_t status = cvHelperSuccess;
    int threshVal = (int) inputs[0];
    int threshMax = (int) inputs[1];
    int threshType = (int) inputs[2];
    try{
        threshold(img, img, threshVal, threshMax, threshType);
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
 * the image. Set the first 5 centroid coordinates to the output values.
 * 
 * @inCount     -> 2
 * @inFormat    -> [Blur Degree (Int), Threshold Value (Int)]
 * 
 * @outCount    -> 2-10
 * @outFormat   -> [CentroidX (Double), CentroidY (Double) ... ]
 */
ADCVStatus_t NDPluginCVHelper::find_centroids(Mat &img, double* inputs, double* outputs){
    static const char* functionName = "find_centroids";
    ADCVStatus_t status = cvHelperSuccess;
    int blurDegree = (int) inputs[0];
    int thresholdVal = (int) inputs[1];
    try{
        GaussianBlur(img, img, Size(blurDegree, blurDegree), 0);
        threshold(img, img, thresholdVal, 255, THRESH_BINARY);
        vector<vector<Point>> contours;
        vector<Vec4i> heirarchy;

        findContours(img, contours, heirarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0,0));
        vector<Moments> contour_moments(contours.size());
        size_t i, j, k, l;
        for(i = 0; i < contours.size(); i++){
            contour_moments[i] = moments(contours[i], false);
        }
        vector<Point2f> contour_centroids(contours.size());
        for(j = 0; j < contours.size(); j++){
            contour_centroids[i] = Point2f((contour_moments[j].m10/contour_moments[j].m00), (contour_moments[j].m01/contour_moments[j].m00));
        }
        for(k = 0; k < contour_centroids.size(); k++){
            if(k%2==0){
                outputs[k] = contour_centroids[k/2].x;
            }
            else{
                outputs[k] = contour_centroids[k/2].y;
            }

            if(k == NUM_OUTPUTS) break;
        }
        for(l = 0; l< contours.size(); i++){
            drawContours(img, contours, l, Scalar(0,255,0), 2, 8, heirarchy, 0, Point());
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
 * Blurs image based on a gaussian kernel
 *
 * @inCount     -> 1
 * @inFormat    -> [blurDegree (Int)]
 *
 * @outCount    -> TODO
 * @outFormat   -> [Param1 (Int), Param2 (Double) ...]
 */
ADCVStatus_t NDPluginCVHelper::gaussian_blur(Mat &img, double* inputs, double* outputs){
    const char* functionName = "gaussian_blur";
    ADCVStatus_t status = cvHelperSuccess;
    int blurDegree = inputs[0];

    try{
        GaussianBlur(img, img, Size(blurDegree, blurDegree), 1, 0, BORDER_DEFAULT);
    }catch(Exception &e){
        print_cv_error(e, functionName);
        status = cvHelperError;
    }
    return status;
}


//------------------------ End of OpenCV wrapper functions -------------------------------------------------


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
        case ADCV_EdgeDetectionCanny:
            status = canny_edge_detection(image, inputs, outputs);
            break;
        case ADCV_Threshold:
            status = threshold_image(image, inputs, outputs);
            break;
        case ADCV_GaussianBlur:
            status = gaussian_blur(image, inputs, outputs);
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


/* Basic constructor/destructor, used by plugin to call processImage */

NDPluginCVHelper::NDPluginCVHelper(){ }

NDPluginCVHelper::~NDPluginCVHelper(){ delete this; }
