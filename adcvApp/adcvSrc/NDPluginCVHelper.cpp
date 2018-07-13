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


/*
 * Simple function that prints OpenCV error information.
 * Used in try/catch blocks
 * 
 * @params: e -> exception thrown by OpenCV function
 */
void NDPluginCVHelper::print_cv_error(Exception &e){
    cout << "OpenCV error: " << e.err << " code: " << e.code << " file: " << e.file << endl;
}


/*
 * Function that does Edge detection using the OpenCV canny function
 * It first blurs the image, then runs the canny function on the resulting
 * blurred image.
 * 
 * @params: img -> the image on which edge detection is to be run
 * @params: threshVal -> value of low threshold
 * @params: threshRatio -> ratio of thresholding, i.e. 1:3 has threshRatio of 3
 * @params: blurDegree -> degree to which image is blurred to remove noise
 * @return: Image with edges detected
 */
Mat NDPluginCVHelper::edge_detector_canny(Mat &img, int threshVal, int threshRatio, int blurDegree){
    Mat temp, detected;
    try{
        blur(img, temp, Size(blurDegree, blurDegree));
    }catch(Exception &e){
        print_cv_error(e);
        return;
    }
    try{
        Canny(temp, temp, threshVal, (threshVal*threshRatio));
    }catch(Exception &e){
        print_cv_error(e);
        return;
    }
    detected = Scalar::all(0);
    img.copyTo(detected, temp);
    imshow("Canny", detected);
    waitKey(0);
    return detected;
}


/*
 * Function that uses the laplacian method of edge detection
 * First uses Gaussian blur to blur the image, then laplacian for edge detection
 * 
 * @params: img -> image on which edge detection will be applied
 * @params: blurDegree -> kernel size for Gaussian Blur
 * @return: image with detected edges.
 */
Mat NDPluginCVHelper::edge_detector_laplacian(Mat &img, int blurDegree){
    Mat temp, detected;
    try{
        GaussianBlur(img, temp, Size(blurDegree, blurDegree),1, 0, BORDER_DEFAULT);
    }catch(Exception &e){
        print_cv_error(e);
        return;
    }
    try{
        Laplacian(temp, temp, CV_16S);
        convertScaleAbs(temp, detected);
    }catch(Exception &e){
        print_cv_error(e);
        return;
    }
    imshow("Laplacian", detected);
    waitKey(0);
    return detected;
}

/*
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
 */
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
    for(int i = 0; i< contours.size; i++){
        drawContours(cropOriginal, contours, i, Scalar(0,255,0), 2, 8, hierarchy, 0, Point());
        circle(cropOriginal, contour_centroids[i], 3, Scalar(255,0,0), -1, 8, 0);
    }
    imshow("contours+centroids", cropOriginal);
    waitKey(0);
    return img;
}

NDPluginCVHelper::NDPluginCVHelper(){
    
}

NDPluginCVHelper::~NDPluginCVHelper(){
    delete this;
}