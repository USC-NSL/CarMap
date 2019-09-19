#!/bin/bash

## Install cmake
sudo apt install cmake
## Install g++
sudo apt install g++

## Install OpenCV 3.4.0
# Install dependencies for OpenCV
sudo apt-get install build-essential
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev

# Build opencv
# git clone https://github.com/opencv/opencv
# cd opencv
# git checkout 3.4.0
# mkdir build && cd build
# cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..
# make -j
# sudo make install

## Build boost libraries
sudo apt install libboost-all-dev
# Build other dependencies
sudo apt install libomp-dev
sudo apt install libeigen3-dev
sudo apt install libncurses5-dev
## Build ORB_SLAM2 dependencies
echo "Configuring and building Thirdparty/DBoW2 ..."
cd ORB_SLAM2/Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
cd ../../g2o
echo "Configuring and building Thirdparty/g2o ..."
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
cd ../../../
# Build libpcl
suod apt install libpcl-dev
