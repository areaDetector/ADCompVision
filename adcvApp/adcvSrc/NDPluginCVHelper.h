/*
 * Header file for helper functions/structs for NDPluginCV
 * 
 * Author: Jakub Wlodek
 * 
 * Created: June 2018
 * 
 */


//include some standard libraries
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <stdio.h>

//OpenCV is used for image manipulation
#include <opencv2/opencv.hpp>

//function that will print information about an opencv error
void print_cv_error(Exception &e);

//function that will perform Canny edge detection
Mat edge_detector_canny(Mat &img, int threshVal, int threshRatio, int blurDegree);

//function that will perform Laplacian edge detection
Mat edge_detector_laplacian(Mat &img, int blurDegree);

//function that will find contours and the centroids of them in a given ROI
Mat centroid_finder(Mat &img, int roiX, int roiY, int roiWidth, int roiHeight, int blurDegree, int threshVal);
