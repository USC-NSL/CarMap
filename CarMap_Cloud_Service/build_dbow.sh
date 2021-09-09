#!/bin/bash

#echo "Configuring and building Thirdparty/DBoW2 ..."
#cd ORB_SLAM2/Thirdparty/DBoW2
#mkdir build
#cd build
#cmake .. -DCMAKE_BUILD_TYPE=Release
#make -j

#cd ../../../
cd ORB_SLAM2/Thirdparty/g2o
echo "Configuring and building Thirdparty/g2o ..."
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

