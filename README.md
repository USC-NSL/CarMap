# CarMap-Fast 3D Feature Map Updates for Automobiles

![](CarMap_Gif.gif)

## Features:
1. CarMap-generated 3D lean map (20x smaller map than ORB-SLAM2)
2. CarMap's dynamic object filter to remove dynamic objects
3. CarMap's semantically annotated 3D map
4. CarMap's position-based robust feature matching
5. CarMap's efficient map update operation
6. CarMap's robust map segment stitching for unmapped regions
7. CarMap-generated 3D map upload/download to the CarMap cloud service
8. Feature map stitching for CarMap generated map

## Build Instructions
There are two directories in the repo:

The vehicle client: **CarMap_Vehicle_Client**

The cloud service: **CarMap_Cloud_Service**

Both of them have similar dependencies and build instructions

To make your life easier, simply download the CarMap docker using:
```
docker pull fawadahm/carmap_nvidia
```

With this, you can skip the text below and move to Configuring CarMap.</br>
### Dependencies:
1) Run the build_carmap.sh script to build the dependencies needed
```
sh build_carmap.sh
```

2) Building OpenCV:
```
git clone https://github.com/opencv/opencv
cd opencv
git checkout 3.4.0
mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..
make -j7
sudo make install
```

3) Eigen 3.1.0 or higher

4) Pangolin
```
git clone https://github.com/stevenlovegrove/Pangolin.git
mkdir build && cd build
cmake ..
make -j
```

5) PCL 1.9.0

### Configuring CarMap:
1) First build CarMap_Cloud_Service
```
cd CarMap_Cloud_Service
bash build_dbow.sh
mkdir build && cd build
cmake ..
make -j
```
2) Build CarMap_Vehicle_Client
```
cd CarMap_Vehicle_Client
bash build_dbow.sh
mkdir build && cd build
cmake ..
make -j
```
3) Download sample dataset for CarMap_Vehicle_Client:
```
mkdir CarMap_Dataset && cd CarMap_Dataset
```
Navigate to the link (https://drive.google.com/uc?export=download&id=1eJ4yZGrQXC50Hp4Idie2I3K5XlyR-Eeb) and download the dataset
```
tar -xz CarMap_Dataset.tar.xz
cd ..
```
### Running CarMap:
1) Run CarMap Cloud Service
```
cd CarMap_Cloud_Service/build/
./server_stitch
cd ../../
```
2) Run CarMap_Vehicle_Client
```
cd CarMap_Vehicle_Client/build/
./main
```

## Instructions for Running CarMap
Both the vehicle client and the cloud service have a Config_File.yaml that dictates the operation mode of CarMap.

### CarMap Cloud Service:
In this section, we describe what some of the fields in Config_File.yaml of CarMap Cloud Service are used for:
1. All the ServerStitch.Path* variables are used to feed input and output paths to CarMap
2. *OneShotStitching*: true for CarMap stitching and false for progressive relocalization
3. *ReceiveMaps*: true for receiving map segments from a vehicle client and false to stitch two maps from disk
4. *CloudServiceIDAddress*: this is the ip address of the cloud service (always starts with ip.)
5. *CloudServicePort*: port address at the cloud service
6. *IntegrateDiff*: true for map update using diff and false for map update via stitch


### CarMap Vehicle Client:
In this section, we describe what some of the fields in Config_File.yaml of CarMap Vehicle Client are used for:
1. *OperationMode.OpCode*:
   * 0 for a first time mapping session with no pre-loaded map
   * 1 for updating a pre-loaded map
   * 2 for using a pre-loaded map without updating it
2. *SaveMap/UpdateMap/LoadMap.StartFrom and StopAt*: starting and ending frame numbers of input dataset
3. Path variables are for input and output paths
4. *Constants*:
   * *RunLocalization*: false for SLAM and true for running only localization (we use false by default)
   * *SaveTrajectory*: true for saving trajectory at output path
   * *IsStereo*: true for stereo camera frames as input
   * *OrbVisualization*: true to visualize the whole CarMap mapping process
5. *Mode*:
   * *RemoveDynamicObjects*: true for using dynamic object filter
   * *LoadDepth*: true to load depth images from input_dataset/depth/ but false by default because we estimate depth from stereo matching
   * *EvaluateSegmentation*: true to evaluate segmentation accuracy using ground truth segmented images
   * *RobustFeatureSearch*: true to enable robust feature matching
  
6. Online Mode Settings:
   * *UploadToCloud*: true to upload map segments/diffs to cloud service
   * *UploadOnlyDiff*: true to upload diff and false to upload whole map segments
   * *CloudIPAddress and CloudPort*: address of carmap cloud service
   * *UploadFrameInterval*: number of frames after which to upload map diff/segments to cloud


## Citation
```bibtex
@inproceedings {carmap,
  author = {Fawad Ahmad and Hang Qiu and Ray Eells and Fan Bai and Ramesh Govindan},
  title = {{CarMap}: Fast 3D Feature Map Updates for Automobiles },
  booktitle = {17th USENIX Symposium on Networked Systems Design and Implementation (NSDI 20)},
  year = {2020},
  isbn = {978-1-939133-13-7},
  address = {Santa Clara, CA},
  pages = {1063--1081},
  url = {https://www.usenix.org/conference/nsdi20/presentation/ahmad},
  publisher = {USENIX Association},
  month = feb,
}
```
