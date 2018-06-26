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
endif


include $(AREA_DETECTOR)/ADCore/ADApp/commonLibraryMakefile

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE