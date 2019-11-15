In this directory you will find a script you may use for building OpenCV.
Just edit the version and architecture at the top and run the script.

You can build ADCompVision with any version of openCV by first running this script,
and then in areaDetector/configure/CONFIG_SITE.local and setting:

OPENCV=$(ADCOMPVISION)/adcvSupport/os/$(EPICS_ARCH)
OPENCV_LIB=$(ADCOMPVISION)/adcvSupport/os/$(EPICS_ARCH)/lib
OPENCV_INCLUDE=$(ADCOMPVISION)/adcvSupport/os/$(EPICS_ARCH)/include

