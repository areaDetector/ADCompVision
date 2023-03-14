# ADCompVision

Author: Jakub Wlodek  
Second Author: Kazimierz Gofron  
Copyright (c): Brookhaven National Laboratory 2018-19  

A plugin that will allow for a comprehensive implementation of Open CV based computer vision intergation with EPICS area detector

Check the RELEASE.md file or the [website](https://areadetector.github.io/areaDetector/ADCompVision/ADCompVision.html) for release notes on current versions.

### Installation

Because ADCompVision is an areaDetector plugin that is separate from ADCore, it requires some setup before use with detector drivers and IOCs.
Follow the below instructions to install ADCompVision:  

**NOTE** ADCompVision depends on OpenCV, so you must install it first from the system package manager or from source from https://github.com/opencv/opencv
```
sudo apt install libopencv-dev
```
---------------------
1. First, navigate to the top level areaDetector repository on your system.
   It should contain ADCore, ADSupport, any device drivers installed, and a configure directory.
2. Next, clone this repository with the command:
   *(Alternatively, you may download a zip or tarball of the sources from the releases section of this repository.)*
```
git clone https://github.com/jwlodek/ADCompVision
```
3. Enter the 'configure' directory at the top level of area detector, and open the RELEASE_PRODS.local file, and add the following:
```
ADCOMPVISION=$(AREA_DETECTOR)/ADCompVision
```
4. Open the CONFIG_SITE.local directory, and ensure that areaDetector builds with OpenCV.
   To do this, find the following code block and make sure that WITH_OPENCV and OPENCV_EXTERNAL are both set to YES.
   If not using a system version of openCV, make sure that the opencv_include and opencv_lib variables are set to the directories with
   opencv include files and compiled binary libraries (.lib/.dll on windows, .so/.a on linux).
```
WITH_OPENCV     = YES
OPENCV_EXTERNAL = YES
#OPENCV_INCLUDE =
#OPENCV_LIB     =
```
5. Next, open CONFIG_SITE.local.EPICS_ARCH, and make sure you set the following.
   If you built OpenCV from source pass the appropriate values for LIB and INCLUDE
```
WITH_OPENCV     = YES
OPENCV          = /usr
#OPENCV_LIB     = $(OPENCV)/lib64
#OPENCV_INCLUDE = $(OPENCV)/include
```
6. We are now done with the configure directory, so back out and enter ADCore/iocBoot.
   Here, open the commonPlugins.cmd file, and add commands to load the NDPluginCV plugin at IOC startup.
```
NDCVConfigure("CV1", $(QSIZE), 0, "$(PORT)", 0, 0, 0, 0, 0, $(MAX_THREADS=5))
dbLoadRecords("$(ADCOMPVISION)/db/NDCV.template", "P=$(PREFIX), R=CV1:, PORT=CV1, ADDR=0, TIMEOUT=1, NDARRAY_PORT=$(PORT), NAME=CV1, NCHANS=$(XSIZE)")
set_requestfile_path("$(ADCOMPVISION)/adcvApp/Db")
```
7. Go back to the top level of ADCore, then enter ADApp, and open the commonDriverMakefile.
   In order to link the correct libraries to the driver during compilation, add the following:
```
ifdef ADCOMPVISION
  $(DBD_NAME)_DBD  += NDPluginCV.dbd
  PROD_LIBS     += NDPluginCV
  ifdef OPENCV_LIB
    opencv_core_DIR += $(OPENCV_LIB)
    PROD_LIBS       += opencv_core opencv_imgproc opencv_highgui opencv_video opencv_videoio
  else
    PROD_SYS_LIBS   += opencv_core opencv_imgproc opencv_highgui opencv_video opencv_videoio
  endif 
endif
```
**NOTE** If using OpenCV 3.* or newer add opencv_imgcodecs to the LIBS.
This is because the highgui package was split into two in the newer versions.
8. The inital setup process is now complete. go back to the top level of ADCore and type:
```
make -sj
```
9. Next, enter ADCompVision and type make, and then the driver you wish to compile, and type make.
10. Finally, prior to running the IOC for the driver, make sure that the path to ADCOMPVISION is set in your EPICS envPaths.  

ADCompVision is now installed and ready for use.

### Setting up the U.I.

Two screens are provided for use with ADCompVision. The first is the main screen for use with the plugin.
It extends the NDPluginBase screen. The second is a line for use with the commonPlugins screen.
Simply add the line to the commonPluginsScreen, and link the 'more' button to the main NDPluginCV screen.
UI screenshots and usage is explained further in the docs:
https://areadetector.github.io/areaDetector/ADCompVision/ADCompVision.html

### Usage

**Certain ADCompVision functions only support 8 bit images, and will thus be downconverted if they are at higher bit depth.
Passthrough will be whatever bit depth required.**

ADCompVision is meant to be a comprehensive implementation of OpenCV functionality into areaDetector.
As a result, because of the quantity of different functionality with different input and output parameters,
having a Process Variable (PV) for every one of them would not have been reasonable.
As a result, the plugin uses 10 generic input PVs and 10 generic output PVs.
Each function accepts different parameters, and relationship between the generic inputs and these parameters can be seen in the user manual.
In addition, when a function is selected, an input and output guide are displayed.
This generic input and output system makes ADCompVision flexible and easy to adapt into any workflow,
as new custom functions can be written and implemented into the plugin without ever touching
the functions that interface with EPICS and areaDetector,
meaning that all that is required to add to the functionality of ADCompVision is a knowledge of C++ and OpenCV.

Further usage and the I/O instruction manual can be found at https://areadetector.github.io/areaDetector/ADCompVision/ADCompVisionManual.html

