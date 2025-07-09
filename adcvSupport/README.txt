In this directory you will find a script you may use for building OpenCV.
Just edit the version and architecture at the top and run the script.

You can build ADCompVision with any version of openCV by first running this script,
and then in areaDetector/configure/CONFIG_SITE.local and setting:

WITH_OPENCV=YES
OPENCV_EXTERNAL=YES
OPENCV=$(ADCOMPVISION)/adcvSupport/os/$(EPICS_ARCH)
OPENCV_LIB=$(ADCOMPVISION)/adcvSupport/os/$(EPICS_ARCH)/lib
OPENCV_INCLUDE=$(ADCOMPVISION)/adcvSupport/os/$(EPICS_ARCH)/include

Where ADCOMPVISION is the location of ADCompVision on your machine

Some things to keep in mind:
When building with non-system opencv, I couldn't get the static
versions of the libraries to link correctly. Thus, in
$(AREA_DETECTOR)/configure/CONFIG_SITE.local.Linux I set
STATIC_BUILD=NO

You may add the custom-built OpenCV to the system path, by building without the -DCMAKE_INSTALL_PREFIX flag,
and running sudo make install. This will place the libraries in /usr/local/lib, which must be added to the ld path.
