/**
 * NDPluginCVHelper.cpp
 * 
 * Helper file for ADCompVision Plugin.
 * 
 * This file will contatin all of the OpenCV wrapper functions.
 * The main plugin will call a function that switches on a PV val,
 * and based on the results, passes the image to the correct helper
 * function. There is no interfacing with EPICS in this file, all of that
 * is done in the NDPluginCV.cpp file
 * 
 * Current functionality includes:
 * 
 * 1. Gaussian Blur
 * 2. Image Thresholding
 * 3. Laplacian edge detection
 * 4. Canny edge detection
 * 5. Centroid Identification
 * 6. User defined function
 * 
 * In-progress functions:
 * 
 * 1. Movement Vectors
 * 2. Object Identification
 *
 * Author: Jakub Wlodek
 * 
 * Created: 26-Jun-2018
 * Last Updated: 14-Jan-2019
 * Copyright (c): Brookhaven National Laboratory 2018-2019
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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#ifdef NDVC_WITH_VIDEO
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>
#endif

using namespace cv;
using namespace std;


const char* libraryName = "NDPluginCVHelper";


// ------------------------ General Utility functions ---------------

/**
 * Simple function that prints OpenCV error information.
 * Used in try/catch blocks
 *
 * @params[in]: e -> exception thrown by OpenCV function
 */
void NDPluginCVHelper::print_cv_error(Exception &e, const char* functionName){
    //cout << "OpenCV error: " << e.err << " code: " << e.code << " file: " << e.file << endl;
    char buff[255];
    char buffshort[255];
    sprintf(buff, "OpenCV Error in function %s: %s code: %d file: %s\n", functionName, e.err.c_str(), e.code, e.file.c_str());
    sprintf(buffshort, "OpenCV Error: %s, Error Code: %d", e.err.c_str(), e.code);
    cvHelperStatus = buffshort;
    printf("%s\n", buff);
}


/**
 * When using the image passthrough feature when no function is selected,
 * if the image is color the BGR format OpenCV uses will conflict with the
 * RGB used by area Detector. This function simply converts everything into RGB
 * 
 * @params[out]: img    -> the input image
 * @return: status
 */
ADCVStatus_t NDPluginCVHelper::fix_coloration(Mat &img){
    const char* functionName = "fix_coloration";
    ADCVStatus_t status = cvHelperSuccess;
    try{
        if(img.channels() == 3){
            cvtColor(img, img, COLOR_BGR2RGB);
        }
        // cvHelperStatus = "Image passed through";
    }catch(Exception &e){
        print_cv_error(e, functionName);
        status = cvHelperError;
    }
    return status;
}



/**
 * Function that takes an image and a set camera depth and scales the image to an 
 * 8 bit image and a certain scaling factor based on camera depth.
 * 
 * @params[in]: img             -> image to scale
 * @params[in]: camera_depth    -> depth of camera (for scaling factor)
 * @return status
 */
ADCVStatus_t NDPluginCVHelper::downscale_image_8bit(Mat &img, ADCVCameraDepth_t camera_depth){
    const char* functionName = "downscale_image_8bit";
    ADCVStatus_t status = cvHelperSuccess;
    try{
        if(img.channels() == 3){
            cvtColor(img, img, COLOR_BGR2GRAY);
        }
        switch(camera_depth){
            case ADCV_8Bit:
                // 8 bit is ok
                break;
            // scale remaining according to: 1/2^(bit depth - 8)
            case ADVC_10Bit:
                img.convertTo(img, CV_8UC1, 1/4.0);
                break;
            case ADCV_12Bit:
                img.convertTo(img, CV_8UC1, 1/16.0);
                break;
            case ADCV_14Bit:
                img.convertTo(img, CV_8UC1, 1/64.0);
                break;
            case ADCV_16Bit:
                img.convertTo(img, CV_8UC1, 1/256.0);
                break;
            default:
                cvHelperStatus = "Camera depth is invalid";
                status = cvHelperError;
                break;
        }
    }catch(Exception &e){
        print_cv_error(e, functionName);
        status = cvHelperError;
    }
    return status;
}


/**
 * Helper function that computes pixel distance between two OpenCV Rect objects
 * 
 * @params[in]: r1  -> The first rectangle
 * @params[in]: r2  -> The second rectangle
 * @return x-pixel distance if it is shorter, y-pixel distance if that is shorter
 */
double NDPluginCVHelper::compute_rect_distance(Rect r1, Rect r2){
    double xDist= -1, yDist = -1;
    if(r1.x > r2.x + r2.width) xDist = r1.x - (r2.x + r2.width);
    else if(r2.x > r1.x + r1.width) xDist = r2.x - (r1.x + r1.width);

    if(r1.y > r2.y + r2.height) yDist = r1.y - (r2.y + r2.height);
    else if(r2.y > r1.y + r1.height) yDist = r2.y - (r1.y + r1.height);

    if(xDist != -1 && yDist != -1) return xDist > yDist ? yDist : xDist;
    else if(xDist != -1) return xDist;
    else return yDist;
}


/**
 * Function that updates the filepath for NDPluginCVHelper
 * 
 * @params[in]: new_filepath    -> new filepath inputted into EPICS
 */
void NDPluginCVHelper::update_str_in(string* new_filepath){
    this->filepath = *new_filepath;
    this->cvHelperStatus = "Updated filepath";
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
        cvHelperStatus = "Finished processing YOURFUNCTION";
    }catch(Exception &e){
        print_cv_error(e, functionName);
        status = cvHelperError;
    }
    return status;
}
*/

//------------- OpenCV function wrapper implementations -------------------


/**
 * WRAPPER  ->  Gaussian Blur
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
    try{
        GaussianBlur(img, img, Size(blurDegree, blurDegree), 1, 0, BORDER_DEFAULT);
        cvHelperStatus = "Computed Gaussian Blur of image";
    }catch(Exception &e){
        print_cv_error(e, functionName);
        status = cvHelperError;
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
 * @outFormat   -> N/A
 */
ADCVStatus_t NDPluginCVHelper::threshold_image(Mat &img, double* inputs, double* outputs){
    const char* functionName = "threshold_image";
    ADCVStatus_t status = cvHelperSuccess;
    int threshVal = (int) inputs[0];
    int threshMax = (int) inputs[1];
    try{
        threshold(img, img, threshVal, threshMax, THRESH_BINARY);
        cvHelperStatus = "Computed image threshold";
    }catch(Exception &e){
        status = cvHelperError;
        print_cv_error(e, functionName);
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
 * @outFormat   -> N/A
 */
ADCVStatus_t NDPluginCVHelper::laplacian_edge_detection(Mat &img, double* inputs, double* outputs){
    const char* functionName = "laplacian_edge_detection";
    int blurDegree = inputs[0];
    int kernel_size = inputs[1];
    int scale = inputs[2];
    int delta = inputs[3];

    ADCVStatus_t status = cvHelperSuccess;
    try{
        GaussianBlur(img, img, Size(blurDegree, blurDegree),1, 0, BORDER_DEFAULT);
        int depth = img.depth();
        Laplacian(img, img, depth, kernel_size, scale, delta, BORDER_DEFAULT);
        convertScaleAbs(img, img);
        cvHelperStatus = "Detected laplacian edges";
    }catch(Exception &e){
        print_cv_error(e, functionName);
        return cvHelperError;
    }
    return status;
}


/**
 * WRAPPER  ->  Sharpen
 * Sharpens image by sutracting Laplacian from blurred image
 *
 * @inCount     -> 4
 * @inFormat    -> [Gaussian blurr (Int), Laplacian kernel size (Int), Laplacian scale (Int), Laplacian delat (Int) ]
 *
 * @outCount    -> 0
 * @outFormat   -> N/A
 */
ADCVStatus_t NDPluginCVHelper::sharpen_images(Mat &img, double* inputs, double* outputs){
    const char* functionName = "sharpen_images";
    int blurDegree = inputs[0]; 
    int kernel_size = inputs[1];
    int scale       = inputs[2];
    int delta       = inputs[3];
    Mat temp;
    
    ADCVStatus_t status = cvHelperSuccess;
    
    try{
        img.copyTo(temp);

        GaussianBlur(img, img, Size(blurDegree, blurDegree),1, 0, BORDER_DEFAULT );
        //cvtColor(img, img, COLOR_BGR2GRAY); // Convert the image to grayscale
        int depth = img.depth();
        Laplacian(img, img, depth, kernel_size, scale, delta, BORDER_DEFAULT );
        convertScaleAbs(img, img);
        cvHelperStatus = "Detected laplacian edges";
                    
        subtract(temp, img, img);
        temp.release();

    }catch(Exception &e){
        print_cv_error(e, functionName);
        return cvHelperError;
    }
    return status;
}


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
        cvHelperStatus = "Detected object edges";
    }catch(Exception &e){
        print_cv_error(e, functionName);
        return cvHelperError;
    }
    return status;
}


/**
 * WRAPPER      -> Subtract Consecutive Images
 * Function that allows the user to take consecutive images recieved from area detector and subtract them
 * in pairs. Reads first image into memory, then waits for second one, when it receives the second one, 
 * subtract them.
 * 
 * @inCount     -> 0
 * @inFormat    -> N/A
 * 
 * @outCount    -> 0
 * @outFormat   -> N/A
 */
ADCVStatus_t NDPluginCVHelper::subtract_consecutive_images(Mat &img, double* inputs, double* outputs){
    const char* functionName = "subtract_consecutive_images";
    ADCVStatus_t status = cvHelperSuccess;
    int order = (int) inputs[0];
    try{
        if(this->wasComputed == false){
            img.copyTo(this->temporaryImg);
            this->wasComputed = true;
            status = cvHelperWait;
            this->cvHelperStatus = "Waiting for next image to subtract";
        }
        else{
            if(order == 1)
                subtract(img, this->temporaryImg, img);
            else
                subtract(this->temporaryImg, img, img);
            this->temporaryImg.release();
            this->wasComputed = false;
            this->cvHelperStatus = "Finished Subtracting images";
        }
    }catch(Exception &e){
        print_cv_error(e, functionName);
        status = cvHelperError;
    }
    return status;
}


/**
 * WRAPPER  ->  ComputeImageStats
 * OpenCV accelerated computation of Image statistics
 *
 * @inCount     -> 0
 * @inFormat    -> N/A
 *
 * @outCount    -> 9
 * @outFormat   -> [total, min, min x, min y, max, max x, max y, mean, sigma]
 */
ADCVStatus_t NDPluginCVHelper::compute_image_stats(Mat &img, double* inputs, double* outputs){
    const char* functionName = "compute_image_stats";
    ADCVStatus_t status = cvHelperSuccess;
    try{
        //int roiX = inputs[0];
        //int roiY = inputs[1];
        //int width = inputs[2];
        //int height = inputs[3];
        Mat temp;
        img.copyTo(temp);
        if(temp.channels() == 3) cvtColor(temp, temp, COLOR_BGR2GRAY);
        Scalar im_sum, mean, sigma;
        double max, min;
        Point min_point, max_point;
        minMaxLoc(temp, &min, &max, &min_point, &max_point);
        im_sum = sum(temp);
        meanStdDev(temp, mean, sigma);

        outputs[0] = *im_sum.val;
        outputs[1] = min;
        outputs[2] = min_point.x;
        outputs[3] = min_point.y;
        outputs[4] = max;
        outputs[5] = max_point.x;
        outputs[6] = max_point.y;
        outputs[7] = *mean.val;
        outputs[8] = *sigma.val;

        temp.release();
        cvHelperStatus = "Finished processing computing image stats";
    }catch(Exception &e){
        print_cv_error(e, functionName);
        status = cvHelperError;
    }
    return status;
}


/**
 * WRAPPER  ->  VideoRecord
 * This function uses the opencv_video and opencv_videoio libraries for writing a video from areaDetector cameras.
 * A valid file path is required. Output video framerate should be set to the camera framerate if a real time video
 * is desired. Supported encodings are: H264, MPEG, DIVX, and LAGS. Not all encodings will be present on each machine,
 * thus some experimentation may be required. The output video will $FILEPATH/CV_Output_Vid_$DATETIME.mp4
 *
 * @inCount     -> 4
 * @inFormat    -> [Framerate (Int), Start/Stop (1 or 0), encoding (1-4), Output File Type (1 or 0)]
 *
 * @outCount    -> 0
 * @outFormat   -> N/A
 */
ADCVStatus_t NDPluginCVHelper::video_record(Mat &img, double* inputs, double* outputs){
    const char* functionName = "video_record";
    ADCVStatus_t status = cvHelperSuccess;
    #ifdef NDCV_WITH_VIDEO
    double outputFramerate  = inputs[0];
    int start_stop          = (int) inputs[1];
    int encoding            = (int) inputs[2];
    int outputType          = (int) inputs[3];
    bool color_bool         = true;
    const char* file_ext    = ".avi";
    if(outputType == 1) file_ext = ".mp4";
    try{
        if(img.channels() != 3) color_bool = false;
        if(!this->isRecording && start_stop == 1){
            time_t t = time(0);
            tm* now = localtime(&t);
            char output_file[256];
            // collect full filename
            sprintf(output_file, "%s/CV_Output_Vid_%d-%d-%d_%d_%d_%d%s", this->filepath.c_str(), 
                (now->tm_year - 100), now->tm_mon + 1, now->tm_mday, now->tm_hour, now->tm_min, now->tm_sec, file_ext);
//            printf("%s\n", output_file);
            this->isRecording = true;
            if(this->filepath == ""){
                this->cvHelperStatus = "Error, no entered file path";
                this->isRecording = false;
            }
            else{
                // select appropriate encoding
                int fourcc = CV_FOURCC('H', '2', '6', '4');
                if(encoding == 1) fourcc = CV_FOURCC('M', 'P', 'E', 'G');
                else if(encoding == 2) fourcc = CV_FOURCC('D', 'I', 'V', 'X');
                else if(encoding == 3) fourcc = CV_FOURCC('m', 'p', '4', 'v');
                else if(encoding == 4) fourcc = CV_FOURCC('M', 'J', 'P', 'G');

		this->video = VideoWriter(output_file, fourcc, outputFramerate, Size(img.cols, img.rows), color_bool);
            }
        }
        if(this->isRecording){
            this->video.write(img);
            if(start_stop == 0){
                // save the video and release memory
                this->isRecording = false;
                this->video.release();
                this->cvHelperStatus = "Finished recording video";
            }
            else{
                this->cvHelperStatus = "Recording...";
            }
        }
    }catch(Exception &e){
        print_cv_error(e, functionName);
        status = cvHelperError;
    }
    #endif
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
        GaussianBlur(img, img, Size(blurDegree, blurDegree), 0);
        threshold(img, img, thresholdVal, 255, THRESH_BINARY);
        vector<vector<Point> > contours;
        vector<Vec4i> heirarchy;

        findContours(img, contours, heirarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0,0));
        if(contours.size() == 0){
            cvHelperStatus = "No contours were found.";
            return cvHelperSuccess;
        }
        else if(contours.size() < numLargestContours){
            numLargestContours = contours.size();
        }
        // clean this part up if possible
        vector<vector<Point> > largestContours(numLargestContours);
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
            contours.erase(contours.begin() + pos);
        }
        vector<Moments> contour_moments(largestContours.size());
        size_t i, j, k, l;
        for(i = 0; i < largestContours.size(); i++){
            contour_moments[i] = moments(largestContours[i], false);
        }
        vector<Point2f> contour_centroids(largestContours.size());
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
        cvHelperStatus = "Calculated Object Centroids";
    } catch(Exception &e){
        status = cvHelperError;
        print_cv_error(e, functionName);
    }
    return status;
}


/**
 * WRAPPER  ->  Movement Vectors (Testing)
 * Function that does feature detection on images a set number of frames apart, and attempts to calculate the 
 * movement vector for the calculated key points. It uses ORB feature detection and vector flow
 * 
 * NOT YET IMPLEMENTED/TESTED
 *
 * @inCount     -> 2
 * @inFormat    -> [Frames Between Images (Int), Num Vectors (Int)]
 *
 * @outCount    -> 0 - 8
 * @outFormat   -> [Vector 1 Start X (Int), Vector 1 Start Y (Int), Vector 1 End X, Vector 1 End Y ...]
 */
ADCVStatus_t NDPluginCVHelper::movement_vectors(Mat &img, double* inputs, double* outputs){
    const char* functionName = "movement_vectors";
    ADCVStatus_t status = cvHelperSuccess;
    int framesBetween = inputs[0];
    int numVectors = inputs[1];
    if(numVectors >2 || numVectors <0){
        return cvHelperError;
    }
    try{
        if(frameCounter == 0){
            //initialize the first starting point image
            img.copyTo(temporaryImg);
            frameCounter++;
            status = cvHelperWait;
        }
        else if(frameCounter < framesBetween){
            frameCounter++;
            status = cvHelperWait;
        }
        else {
            frameCounter = 0;
            //calc movement vectors.
        }
        cvHelperStatus = "Processed Movement Vectors";
    }catch(Exception &e){
        print_cv_error(e, functionName);
        status = cvHelperError;
    }
    return status;
}


/**
 * WRAPPER  ->  Object Identification (Testing)
 * Function that detects contours in an image and returns information regarding said contours
 * 
 * NOT YET IMPLEMENTED/TESTED
 *
 * @inCount     -> 4
 * @inFormat    -> [Param1 (Int), Param2 (Double) ...]
 *
 * @outCount    -> 10
 * @outFormat   -> [Param1 (Int), Param2 (Double) ...]
 */
ADCVStatus_t NDPluginCVHelper::obj_identification(Mat &img, double* inputs, double* outputs){
    const char* functionName = "obj_identification";
    ADCVStatus_t status = cvHelperSuccess;
    int upperSizeThreshold = inputs[0];
    int lowerSizeThreshold = inputs[1];
    int blurDegree = inputs[2];
    int thresholdVal = inputs[3];

    try{
        // first we need to convert to grayscale if necessary
        if(img.channels()==3){
            cvtColor(img, img, COLOR_BGR2GRAY);
        }
        GaussianBlur(img, img, Size(blurDegree, blurDegree), 0);
        threshold(img, img, thresholdVal, 255, THRESH_BINARY);
        vector<vector<Point> > contours;
        vector<vector<Point> > validContours;
        vector<Vec4i> heirarchy;
        findContours(img, contours, heirarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0,0));
        // TODO
        size_t k;
        for(k = 0; k< contours.size(); k++){
            if(contourArea(contours[k]) > lowerSizeThreshold && contourArea(contours[k]) < upperSizeThreshold){
                validContours.push_back(contours[k]);
            }
        }
        if(validContours.size()==0){
            cvHelperStatus = "No contours within thresholds found";
            return cvHelperSuccess;
        }
        outputs[0] = contourArea(validContours[0]);
        //outputs[1] = cvContourPerimeter(validContours[0]);
        //outputs[2] = math.pow((4*outputs[0])/math.pi, 0.5);
        cvHelperStatus = "Finished object identification";
    }catch(Exception &e){
        print_cv_error(e, functionName);
        status = cvHelperError;
    }
    return status;
}

/**
 * WRAPPER  ->  Median Blur
 * Function that applies a median blur to an image. A median blur is a non-linear filter that replaces each pixel
 * with the median value of the pixels in a square kernel around it. This is useful for removing noise from an image
 * while preserving edges. The kernel size must be an odd number.
 *
 * @inCount     -> 1
 * @inFormat    -> [Kernel size (Odd Int)]
 *
 * @outCount    -> 0
 * @outFormat   -> []
 */
ADCVStatus_t NDPluginCVHelper::median_blur(Mat &img, double* inputs, double* outputs){
    const char* functionName = "median_blur";
    ADCVStatus_t status = cvHelperSuccess;
    try
    {
        int ksize = static_cast<int>(inputs[0]);
        medianBlur(img,img,ksize);
        cvHelperStatus = "Filtered input image";
    }
    catch(Exception& e)
    {
        print_cv_error(e, functionName);
        status = cvHelperError;
    }
    return status;
}

/**
 * WRAPPER  ->  User Function
 * This is an unimplemented wrapper function that has already been added to the PV database in order to
 * simplify creating user defined functions. Simply implement this function and its description function,
 * and then select 'User Function' in function set 3. 
 *
 * @inCount     -> n
 * @inFormat    -> [Param1 (Int), Param2 (Double) ...]
 *
 * @outCount    -> n
 * @outFormat   -> [Param1 (Int), Param2 (Double) ...]
 */
ADCVStatus_t NDPluginCVHelper::user_function(Mat &img, double* inputs, double* outputs){
    const char* functionName = "user_function";
    ADCVStatus_t status = cvHelperSuccess;
    try{
        // Process your image here
        cvHelperStatus = "Finished processing user defined function";
    }catch(Exception &e){
        print_cv_error(e, functionName);
        status = cvHelperError;
    }
    return status;
}


/**
 * WRAPPER  ->  Distance Between Contours
 * Function that computes bounding boxes between the two largest computed contours in the image,
 * checks the distance between them and sends an alarm if they are within a distance threshold.
 *
 * @inCount     -> 5
 * @inFormat    -> [[Distance Threshold (Int), Blur Kernel Size (Int), Threshold (Int), Apply Blur (Toggle), Pixel Size Threshold (Int)]
 *
 * @outCount    -> 2
 * @outFormat   -> [Is Within Threshold (Binary Int), Distance in Pixels (Int)]
 */
ADCVStatus_t NDPluginCVHelper::distance_between_ctrs(Mat &img, double* inputs, double* outputs){
    const char* functionName = "distance_between_ctrs";
    ADCVStatus_t status = cvHelperSuccess;
    int distance_threshold = inputs[0];
    int blurDegree = inputs[1];
    int thresholdVal = inputs[2];
    int applyBlur = inputs[3];
    int contour_size_thresh = inputs[4];
    Mat temp;

    try{
        int largest_area = 0;
        int second_area = 0;
        Rect lbounding_rect, sbounding_rect;
        if(applyBlur > 0) GaussianBlur(img, temp, Size(blurDegree, blurDegree), 0);
        if(img.channels() == 3) cvtColor(temp, temp, COLOR_BGR2GRAY);
        threshold(temp, temp, thresholdVal, 255, THRESH_BINARY);
        vector<vector<Point> > contours;
        findContours(temp, contours, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0,0));
        if(contours.size() > 1){
            for(unsigned int i = 0; i< contours.size(); i++){
                double area = contourArea(contours[i]);
                if(area < contour_size_thresh){
                    if(area > largest_area){
                        if(largest_area > second_area){
                            second_area = largest_area;
                            sbounding_rect = lbounding_rect;
                        }
                        largest_area = area;
                        lbounding_rect = boundingRect(contours[i]);
                    }
                    else if( area > second_area){
                        second_area = area;
                        sbounding_rect = boundingRect(contours[i]);
                    }
                }
            }
            cv::rectangle(img, lbounding_rect, Scalar(0, 255, 0), 3);
            cv::rectangle(img, sbounding_rect, Scalar(0, 255, 0), 3);
            outputs[1] = compute_rect_distance(lbounding_rect, sbounding_rect);
            if(outputs[1] < distance_threshold) outputs[0] = 1;
            else outputs[0] = 0;
            cvHelperStatus = "Finished processing distance between contours";
        }
        else cvHelperStatus = "Couldn't identify two or more contours";
        temp.release();
    }catch(Exception &e){
        print_cv_error(e, functionName);
        status = cvHelperError;
    }
    return status;
}


/**
 * WRAPPER  ->  Convert Image Format
 * Converts the format of the image to a different one for use with other AD Plugins. This is useful for 
 * cameras that only support one format but a different one is required, ex. ADPluginDmtx needs 8bit rgb image, so 
 * grayscale camera needs to be converted.
 *
 * @inCount     -> 2
 * @inFormat    -> [To grayscale (Toggle), To rgb (Toggle)]
 *
 * @outCount    -> 0
 * @outFormat   -> N/A
 */
ADCVStatus_t NDPluginCVHelper::convert_image_format(Mat &img, double* inputs, double* outputs){
    const char* functionName = "convert_image_format";
    ADCVStatus_t status = cvHelperSuccess;
    int toGray = inputs[0];
    int toRGB = inputs[1];
    try{
        if(toGray == 1 && img.channels() == 3) cvtColor(img, img, COLOR_BGR2GRAY);
        else if(toRGB == 1 && img.channels() == 1) cvtColor(img, img, COLOR_GRAY2RGB);

        cvHelperStatus = "Finished processing convert_image_format";
    }catch(Exception &e){
        print_cv_error(e, functionName);
        status = cvHelperError;
    }
    return status;
}

/**
 * WRAPPER  ->  Log scaling
 *
 * Converts the format of the image to a different one for use with other AD Plugins. This is useful for 
 * cameras that only support one format but a different one is required, ex. ADPluginDmtx needs 8bit rgb image, so 
 * grayscale camera needs to be converted.
 *
 * @inCount     -> 2
 * @inFormat    -> [min, max]
 *
 * @outCount    -> 0
 * @outFormat   -> N/A
 */
ADCVStatus_t NDPluginCVHelper::log_scaling(Mat &img, double* inputs, double* outputs){
    const char* functionName = "log_scaling";
    ADCVStatus_t status = cvHelperSuccess;
    int minValue = inputs[0];
    int outputBitDepth = inputs[1];
    try{
	if(img.channels() == 3) cvtColor(img, img, COLOR_BGR2GRAY);
	subtract(img, Scalar(minValue), img);
	img.convertTo(img, CV_64F);
	img = img + 1;
	log(img, img);
	
	switch(outputBitDepth){
            case 0:
	        normalize(img, img, 0, 255, cv::NORM_MINMAX);
        	img.convertTo(img, CV_8U);
		break;
	    case 1:
	        normalize(img, img, 0, 65535, cv::NORM_MINMAX);
        	img.convertTo(img, CV_16U);
		break;
	    default:
	        normalize(img, img, 0, 255, cv::NORM_MINMAX);
        	img.convertTo(img, CV_8U);	
		break;
	}
	cvHelperStatus = "Finished computing log scaled image";
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
    *description = "Description of YOURFUNCTION";
    populate_remaining_descriptions(inputDesc, outputDesc, numInput, numOutput);
    return status;
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
    cvHelperStatus = "Populated Input and output descriptions";
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
 * @params[out]: outputDesc     -> array of output    else cvHelperStatus = "Image processed successfully"; descriptions
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
    int numInput = 4;
    int numOutput = 0;
    inputDesc[0] = "Gauss Blurr degree [int]";
    inputDesc[1] = "Laplace kernel size [int]";
    inputDesc[2] = "Laplace scale [int]";
    inputDesc[3] = "Laplace delat [int]";

    *description = "Edge detection using a combination of a Gaussian Blur kernel and a Laplacian kernel";
    populate_remaining_descriptions(inputDesc, outputDesc, numInput, numOutput);
    return status;
}


/**
 * Function that sets the I/O descriptions for Distance between check
 * 
 * @params[out]: inputDesc      -> array of input descriptions
 * @params[out]: outputDesc     -> array of output descriptions
 * @params[out]: description    -> overall function usage description
 * @return: void
 */
ADCVStatus_t NDPluginCVHelper::get_dist_between_description(string* inputDesc, string* outputDesc, string* description){
    ADCVStatus_t status = cvHelperSuccess;
    int numInput = 5;
    int numOutput = 2;
    inputDesc[0] = "Distance Threshold [int]";
    inputDesc[1] = "Blur Kernel Size [Odd int]";
    inputDesc[2] = "Threshold Value [int < 255]";
    inputDesc[3] = "Apply Blur [1 = yes, 0 = no]";
    inputDesc[4] = "Size Threshold [int]";
    outputDesc[0] = "Within Threshold Alarm";
    outputDesc[1] = "Detected Min Pixel dist";
    *description = "Description of Computes distance between contours";
    populate_remaining_descriptions(inputDesc, outputDesc, numInput, numOutput);
    return status;
}




/*
 * Function that sets the I/O descriptions for Sharpen
 * 
 * @params[out]: inputDesc      -> array of input descriptions
 * @params[out]: outputDesc     -> array of output descriptions
 * @params[out]: description    -> overall function usage description
 * @return: void
 */
ADCVStatus_t NDPluginCVHelper::get_sharpen_description(string* inputDesc, string* outputDesc, string* description){
    ADCVStatus_t status = cvHelperSuccess;
    int numInput = 4;
    int numOutput = 0;
    inputDesc[0] = "Gauss Blurr degree [int]";
    inputDesc[1] = "Laplace kernel size [int]";
    inputDesc[2] = "Laplace scale [int]";
    inputDesc[3] = "Laplace delta [int]";

    // outputDesc[0] = "Output 1 Description";
    // outputDesc[1] = "Output 2 Description";
 
    *description = "Sharpen images using laplacian";
    populate_remaining_descriptions(inputDesc, outputDesc, numInput, numOutput);
    return status;
}

/**
 * Function that sets the I/O descriptions for image subtraction
 * 
 * @params[out]: inputDesc      -> array of input descriptions
 * @params[out]: outputDesc     -> array of output descriptions
 * @params[out]: description    -> overall function usage description
 * @return: void
 */
ADCVStatus_t NDPluginCVHelper::get_subtract_description(string* inputDesc, string* outputDesc, string* description){
    ADCVStatus_t status = cvHelperSuccess;
    int numInput = 1;
    int numOutput = 0;
    inputDesc[0] = "2nd - 1st(0)/1st - 2nd(1)";
    *description = "Subtracts consecutive images. Used in certain implementations of Dual Thresholding";
    populate_remaining_descriptions(inputDesc, outputDesc, numInput, numOutput);
    return status;
}


/**
 * Function that sets the I/O descriptions for Image Stats
 * 
 * @params[out]: inputDesc      -> array of input descriptions
 * @params[out]: outputDesc     -> array of output descriptions
 * @params[out]: description    -> overall function usage description
 * @return: void
 */
ADCVStatus_t NDPluginCVHelper::get_image_stats_description(string* inputDesc, string* outputDesc, string* description){
    ADCVStatus_t status = cvHelperSuccess;
    int numInput = 0;
    int numOutput = 9;
    outputDesc[0] = "Total";
    outputDesc[1] = "Minimum";
    outputDesc[2] = "Min X";
    outputDesc[3] = "Min Y";
    outputDesc[4] = "Maximum";
    outputDesc[5] = "Max X";
    outputDesc[6] = "Max Y";
    outputDesc[7] = "Mean";
    outputDesc[8] = "Sigma";
    *description = "Compute image statistics";
    populate_remaining_descriptions(inputDesc, outputDesc, numInput, numOutput);
    return status;
}


/**
 * Function that sets the I/O descriptions for Canny Edge Detection
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
 * Function that sets the I/O descriptions for Median Filter
 * 
 * @params[out]: inputDesc      -> array of input descriptions
 * @params[out]: outputDesc     -> array of output descriptions
 * @params[out]: description    -> overall function usage description
 * @return: void
 */
ADCVStatus_t NDPluginCVHelper::get_median_blur_description(string* inputDesc, string* outputDesc, string* description){
    ADCVStatus_t status = cvHelperSuccess;
    int numInput = 1;
    int numOutput = 0;
    inputDesc[0] = "Window Size (px)";
    *description = "Applies median value of n-by-n window";
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
 * Function that sets the I/O descriptions for movement vectors
 * 
 * @params[out]: inputDesc      -> array of input descriptions
 * @params[out]: outputDesc     -> array of output descriptions
 * @params[out]: description    -> overall function usage description
 * @return: void
 */
ADCVStatus_t NDPluginCVHelper::get_movement_vectors_description(string* inputDesc, string* outputDesc, string* description){
    ADCVStatus_t status = cvHelperSuccess;
    int numInput = 2;
    int numOutput = 8;
    inputDesc[0] = "Frames Between check (Int)";
    inputDesc[1] = "Max Vectors counted (Int 1 or 2)";
    outputDesc[0] = "Vector 1 Start X";
    outputDesc[1] = "Vector 1 Start Y";
    outputDesc[2] = "Vector 1 End X";
    outputDesc[3] = "Vector 1 End Y";
    outputDesc[4] = "Vector 2 Start X";
    outputDesc[5] = "Vector 2 Start Y";
    outputDesc[6] = "Vector 2 End X";
    outputDesc[7] = "Vector 2 End Y";
    *description = "Function that tracks movement of objects between two images separated by a certain number of frames. Not fully implemented";
    populate_remaining_descriptions(inputDesc, outputDesc, numInput, numOutput);
    return status;
}


/**
 *
 * Function that sets the I/O descriptions for Object Identification
 * 
 * @params[out]: inputDesc      -> array of input descriptions
 * @params[out]: outputDesc     -> array of output descriptions
 * @params[out]: description    -> overall function usage description
 * @return: void
 */
ADCVStatus_t NDPluginCVHelper::get_obj_identification_description(string* inputDesc, string* outputDesc, string* description){
    ADCVStatus_t status = cvHelperSuccess;
    int numInput = 4;
    int numOutput = 10;
    inputDesc[0] = "Upper Cntr Size Thresh";
    inputDesc[1] = "Lower Cntr Size Thresh";
    inputDesc[2] = "Blur Degree";
    inputDesc[3] = "Threshold Val";
    outputDesc[0] = "Contour Area";
    outputDesc[1] = "Contour Perimeter";
    outputDesc[2] = "Equivalent Diameter";
    outputDesc[3] = "Solidity";
    outputDesc[4] = "Aspect Ratio";
    outputDesc[5] = "Contour Extent";
    outputDesc[6] = "Min Enclosing Circle Rad";
    outputDesc[7] = "Min Enclosing Circle X";
    outputDesc[8] = "Min Enclosing Circle Y";
    outputDesc[9] = "Orientation";
    *description = "Identify object contours and list information";
    populate_remaining_descriptions(inputDesc, outputDesc, numInput, numOutput);
    return status;
}


/**
 * Function that sets the I/O descriptions for a user defined function
 * 
 * @params[out]: inputDesc      -> array of input descriptions
 * @params[out]: outputDesc     -> array of output descriptions
 * @params[out]: description    -> overall function usage description
 * @return: void
 */
ADCVStatus_t NDPluginCVHelper::get_user_function_description(string* inputDesc, string* outputDesc, string* description){
    ADCVStatus_t status = cvHelperSuccess;
    int numInput = 0;
    int numOutput = 0;
    *description = "Describe what your function does here";
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


/**
 * Function that sets the I/O descriptions for convert_image _format
 * 
 * @params[out]: inputDesc      -> array of input descriptions
 * @params[out]: outputDesc     -> array of output descriptions
 * @params[out]: description    -> overall function usage description
 * @return: void
 */
ADCVStatus_t NDPluginCVHelper::get_convert_format_descripton(string* inputDesc, string* outputDesc, string* description){
    ADCVStatus_t status = cvHelperSuccess;
    int numInput = 2;
    int numOutput = 0;
    inputDesc[0] = "Toggle toGrayscale (0, 1)";
    inputDesc[1] = "Toggle toRGB (0, 1)";
    *description = "Converts image into a different color mode fomat";
    populate_remaining_descriptions(inputDesc, outputDesc, numInput, numOutput);
    return status;
}


/**
 * Function that sets the I/O descriptions for video_record
 * 
 * @params[out]: inputDesc      -> array of input descriptions
 * @params[out]: outputDesc     -> array of output descriptions
 * @params[out]: description    -> overall function usage description
 * @return: void
 */
ADCVStatus_t NDPluginCVHelper::get_video_record_description(string* inputDesc, string* outputDesc, string* description){
    ADCVStatus_t status = cvHelperSuccess;
    int numInput = 0;
    int numOutput = 0;
    *description = "Video functions not built into ADCompVision. Set OPENCV_WITH_VIDEO=YES in Makefile and rebuild.";
    #ifdef NDCV_WITH_VIDEO
    numInput = 4;
    inputDesc[0] = "Output Framerate (Int)";
    inputDesc[1] = "Start(1)/Stop(0)";
    inputDesc[2] = "H264/MPEG/DIVX/MP4/MJPG (0-4)";
    inputDesc[3] = ".avi(0)/.mp4(1)";
    *description = "Function that allows for recording video using areaDetector. Requires valid file path";
    #endif
    populate_remaining_descriptions(inputDesc, outputDesc, numInput, numOutput);
    return status;
}

/**
 * Function that sets the I/O descriptions for log_scaling
 * 
 * @params[out]: inputDesc      -> array of input descriptions
 * @params[out]: outputDesc     -> array of output descriptions
 * @params[out]: description    -> overall function usage description
 * @return: void
 */
ADCVStatus_t NDPluginCVHelper::get_log_scaling_description(string* inputDesc, string* outputDesc, string* description){
    ADCVStatus_t status = cvHelperSuccess;
    int numInput = 2;
    int numOutput = 0;
    *description = "Perform logarithmic scaling of the input image";
    inputDesc[0] = "Minimum level";
    inputDesc[1] = "Bit depth (0 = 8 / 1 = 16)";
    populate_remaining_descriptions(inputDesc, outputDesc, numInput, numOutput);
    return status;
}
/* ---------------------- Functions called from the EPICS Plugin implementation ----------------------- */


/**
 * Function that is called from the ADCompVision plugin. It detects which function is being requested, and calls the appropriate
 * opencv wrapper function from those above.
 * 
 * @params[out]: image       -> pointer to Mat object
 * @params[in]:  function    -> type of CV function to perform
 * @params[in]:  inputs      -> array with inputs for functions
 * @params[out]: outputs     -> array for outputs of functions
 * @return: status      -> check if library function completed successfully
 */
ADCVStatus_t NDPluginCVHelper::processImage(Mat &image, ADCVFunction_t function, ADCVCameraDepth_t camera_depth, double* inputs, double* outputs){
    const char* functionName = "processImage";
    ADCVStatus_t status;

    switch(function){
        case ADCV_Threshold:
            status = downscale_image_8bit(image, camera_depth);
            status = threshold_image(image, inputs, outputs);
            break;
        case ADCV_GaussianBlur:
            status = gaussian_blur(image, inputs, outputs);
            break;
        case ADCV_EdgeDetectionCanny:
            status = downscale_image_8bit(image, camera_depth);
            status = canny_edge_detection(image, inputs, outputs);
            break;
        case ADCV_MedianBlur:
            status = downscale_image_8bit(image, camera_depth);
            status = median_blur(image, inputs, outputs);
            break;
        case ADCV_CentroidFinder:
            status = downscale_image_8bit(image, camera_depth);
            status = find_centroids(image, inputs, outputs);
            break;
        case ADCV_Laplacian:
            status = downscale_image_8bit(image, camera_depth);
            status = laplacian_edge_detection(image, inputs, outputs);
            break;
	case ADCV_LogScaling:
	    status = log_scaling(image, inputs, outputs);
	    break;
        case ADCV_Sharpen:
            status = downscale_image_8bit(image, camera_depth);
            status = sharpen_images(image, inputs, outputs);
            break;
        case ADCV_ConvertFormat:
            status = downscale_image_8bit(image, camera_depth);
            status = convert_image_format(image, inputs, outputs);
            break;
        case ADCV_Subtract:
            status = subtract_consecutive_images(image, inputs, outputs);
            break;
        /*
        case ADCV_MovementVectors:
            status = movement_vectors(image, inputs, outputs);
            break;
        */
        case ADCV_ImageStats:
            status = compute_image_stats(image, inputs, outputs);
            break;
        case ADCV_UserDefined:
            status = user_function(image, inputs, outputs);
            break;
        case ADCV_DistanceCheck:
            //status = downscale_image_8bit(image, camera_depth);
            status = distance_between_ctrs(image, inputs, outputs);
            break;
        case ADCV_VideoRecord:
            status = video_record(image, inputs, outputs);
            break;
        default:
            status = cvHelperError;
            break;
    }

    fix_coloration(image);

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
    //const char* functionName = "getFunctionDescription";
    ADCVStatus_t status;

    switch(function){
        case ADCV_Threshold:
            status = get_threshold_description(inputDesc, outputDesc, description);
            break;
        case ADCV_GaussianBlur:
            status = get_gaussian_blur_description(inputDesc, outputDesc, description);
            break;
        case ADCV_Subtract:
            status = get_subtract_description(inputDesc, outputDesc, description);
            break;
        case ADCV_Laplacian:
            status = get_laplacian_description(inputDesc, outputDesc, description);
            break;
        case ADCV_EdgeDetectionCanny:
            status = get_canny_edge_description(inputDesc, outputDesc, description);
            break;
        case ADCV_MedianBlur:
            status = get_median_blur_description(inputDesc, outputDesc, description);
            break;
        case ADCV_CentroidFinder:
            status = get_centroid_finder_description(inputDesc, outputDesc, description);
            break;
        case ADCV_Sharpen:
            status = get_sharpen_description(inputDesc, outputDesc, description);
            break;
        case ADCV_ConvertFormat:
            status = get_convert_format_descripton(inputDesc, outputDesc, description);
            break;
	case ADCV_LogScaling:
	    status = get_log_scaling_description(inputDesc, outputDesc, description);
	    break;
        /*
        case ADCV_MovementVectors:
            status = get_movement_vectors_description(inputDesc, outputDesc, description);
            break;
        */
        case ADCV_ImageStats:
            status = get_image_stats_description(inputDesc, outputDesc, description);
            break;
        case ADCV_UserDefined:
            status = get_user_function_description(inputDesc, outputDesc, description);
            break;
        case ADCV_DistanceCheck:
            status = get_dist_between_description(inputDesc, outputDesc, description);
            break;
        case ADCV_VideoRecord:
            status = get_video_record_description(inputDesc, outputDesc, description);
            break;
        default:
            status = get_default_description(inputDesc, outputDesc, description);
            break;
    }
    if(function == ADCV_NoFunction) cvHelperStatus = "No function selected";
    else if(status == cvHelperError){
        cvHelperStatus = "Error, Function does not support I/O descriptions";
    }
    return status;
}

/* Basic constructor/destructor, used by plugin to call processImage */

NDPluginCVHelper::NDPluginCVHelper(){ }

NDPluginCVHelper::~NDPluginCVHelper(){ }
