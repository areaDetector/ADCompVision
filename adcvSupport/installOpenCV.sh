#!/bin/bash

# Set version and architecture
EPICS_ARCH=linux-x86_64
OPENCV_VERSION=3.2.0


git clone https://github.com/opencv/opencv
mkdir os
mkdir os/$EPICS_ARCH
cd opencv
git checkout -q $OPENCV_VERSION
mkdir build
cd build
# Remove the Install prefix flag to install to /usr/local/lib
cmake  -DWITH_OPENMP=OFF -DWITH_TBB=OFF -DWITH_IPP=OFF -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF -DBUILD_DOCS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_TESTS=OFF -DBUILD_UNIT_TESTS=OFF -DCMAKE_INSTALL_PREFIX=../../os/$EPICS_ARCH ..
make -j 8
make install
