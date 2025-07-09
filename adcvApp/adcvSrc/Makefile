#Author: Jakub Wlodek June 2018

TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# The following gets rid of the -fno-implicit-templates flag on vxWorks,
# so we get automatic template instantiation.
# This is what we want for the source files that define template functions.
ifeq (vxWorks,$(findstring vxWorks, $(T_A)))
    CODE_CXXFLAGS=
endif

DBD += NDPluginCV.dbd

INC += NDPluginCV.h
INC += NDPluginCVHelper.h

LIBRARY_IOC += NDPluginCV

NDPluginCV_SRCS += NDPluginCV.cpp
NDPluginCV_SRCS += NDPluginCVHelper.cpp

#Must link Opencv here

ifdef OPENCV_INCLUDE
    USR_INCLUDES += -I$(OPENCV_INCLUDE)
else
ifeq (linux-x86_64, $(findstring linux-x86_64, $(T_A)))
# for opencv4 on linux, we need this deeper include search
USR_INCLUDES += -I/usr/include/opencv4
endif
endif

ifdef OPENCV_LIB
    NDPluginCV_DIR += $(OPENCV_LIB)
    opencv_core_DIR = $(OPENCV_LIB)
    opencv_imgproc_DIR = $(OPENCV_LIB)
    opencv_highgui_DIR = $(OPENCV_LIB)

    LIB_LIBS += opencv_core opencv_imgproc
else
    LIB_SYS_LIBS += opencv_core opencv_imgproc
endif

ifeq ($(WITH_OPENCV_VIDEO), YES)
    USR_CPPFLAGS += -D NDCV_WITH_VIDEO
    ifdef OPENCV_LIB
        opencv_video_DIR = $(OPENCV_LIB)
        opencv_videoio_DIR = $(OPENCV_LIB)

        LIB_LIBS += opencv_video opencv_videoio
    else
        LIB_SYS_LIBS += opencv_video opencv_videoio
    endif

endif

include $(AREA_DETECTOR)/ADCore/ADApp/commonLibraryMakefile

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
