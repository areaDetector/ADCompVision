#!/bin/bash

EPICS_ARCH=linux-x86_64
OPENCV_VERSION=3.2.0


git clone https://github.com/opencv/opencv
mkdir os
mkdir os/$EPICS_ARCH
cd opencv
git checkout -q $OPENCV_VERSION
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../../os/$EPICS_ARCH ..
make -j 4
make install