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
endif
ifdef OPENCV_LIB
    NDPluginCV_DIR += $(OPENCV_LIB)
    opencv_core_DIR = $(OPENCV_LIB)
    opencv_imgproc_DIR = $(OPENCV_LIB)
    opencv_highgui_DIR = $(OPENCV_LIB)

    LIB_LIBS += opencv_core opencv_imgproc opencv_highgui
else
    LIB_SYS_LIBS += opencv_core opencv_imgproc opencv_highgui
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
