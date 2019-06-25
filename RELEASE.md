# ADCompVision Releases

Author: Jakub Wlodek
Corresponding Author: Kazimierz Gofron

Dependencies: ADCompVision requires opencv core, highgui, image processing, and image codecs, opencv_video, opencv_videoio.

# Release Notes

<!--RELEASE START-->

## R1-2 (???-June-2019)

* Computer Vision functions implemented
    * Video Record - allows for video recording with areaDetector
    * Convert Format - Converts color format and Data type of input image.

* Additional Feature changes
    * Added filepath PV
    * Added filepath exists PV
    * Added opencv_video and opencv_videoio as library dependancies
    * New medm, edm, and adl screens (Courtesy of Mark Rivers)
    * New rst documentation pages (Courtesy of Mark Rivers

* Bug Fixes
    * Edited mat2NDArray function to remove potential memory leak
    * Comment + documentation updates

## R1-1 (17-April-2019)

* Computer Vision functions implemented:
    * Sharpening filter
    * Image subtraction
    * Image statistics
    * Distance between objects

* Additional Feature changes
    * File saving temporarily removed due to crashing issues over ssh and additional dependency
    * Camera bit depth selector added to play better with higher bit images
    * Image scaling corrected to account for other bit depth images
    * Minor screen updates to reflect feature changes

* Bugs Fixed
    * Bug where invalid bit depth could cause IOC to crash
    * Bug where certain PV would cause error at IOC startup
    * Fixed Image passthrough to work with any bit depth
    * Fixed colorspace of color images passed through
    * Removed code that caused certain compiler warnings - Cleaner compile

## R1-0 (14-January-2019)

* Computer Vision functions implemented:
    * Gaussian Blur
    * Thresholding
    * Laplacian Edge Detection
    * Canny Edge Detection
    * Centroid Detection
    * User Definable Function

* Additional Features added
    * Processed image file saving. (This can work in tandem with NDPluginFile)
    * Support for mono or RGB images
    * Support for 8, 16, 32, and 64 bit images
    * Flexible and modular function implementation system
    * CSS User Interface screen
    * Documentation

* Known Limitations:
    * No current support for performing operations accross multiple images. Required for several CV functions
    * Conversion between PV and ADCV_Function is convoluted and should be reworked

* Future Release Plans
    * Expand list of supported functions
        * Motion Vectors
        * Object identification
        * Image alignment
        * More...
    * Perform tests with variety of cameras
    * Performance improvements and bug fixes
    * Expand file saving to allow for capture, stream, etc