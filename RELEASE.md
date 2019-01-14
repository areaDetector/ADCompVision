# ADCompVision Releases

Author: Jakub Wlodek
Corresponding Author: Kazimierz Gofron

Dependencies: ADCompVision requires opencv core, highgui, image processing, and image codecs.

# Release Notes

<!--RELEASE START-->

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