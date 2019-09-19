//
// Created by on 9/6/17.
//

#ifndef CROWDSOURCED_HD_MAP_ZEDINITIALIZATION_H
#define CROWDSOURCED_HD_MAP_ZEDINITIALIZATION_H

//#include <sl/Camera.hpp>
#include <System.h>
#include <fstream>
#include<opencv2/core/core.hpp>
//#include <ORBmatcher.h>

std::string trajectoryFilePath;
ofstream trajectoryFile;
bool isTracking;
bool isSaveTrajectory;


//using namespace sl;

//Initialize ZED Camera
//int initializeZED (Camera *, InitParameters *, bool, sl::String);
//Initialize ZED SLAM
//void initializeZEDSLAM (Camera *, TrackingParameters *, String, bool , String = NULL);

//SL Mat to CV Mat conversion
//cv::Mat slMat2cvMat(sl::Mat&);


//ORB SLAM2 Initialization
ORB_SLAM2::System *initializeORBSlam (string vocabFile, string settingFile, bool visualization, bool saveMap, bool reuseMap, string pathToLoadMapFile, bool runLocalizationMode, bool saveTrajectory, string pathToTrajectoryFile, bool isStereo, const string serverAddr, const int portNumber, bool isUploadMap, string viewerWindowName, bool reconstructMap, bool isStitchMode, bool isDynamicObjectRemoval);
ORB_SLAM2::System *initializeORBSlam (string vocabFile, string settingFile, bool visualization, bool saveMap, bool reuseMap, string pathToLoadMapFile, bool runLocalizationMode, bool saveTrajectory, string pathToTrajectoryFile, bool isStereo, const string serverAddr, const int portNumber, bool isUploadMap, string viewerWindowName, bool reconstructMap, bool isStitchMode, bool isDynamicObjectRemoval, bool isMajorityVoting, bool isRobustFeatureSearch);
ORB_SLAM2::System *initialize_ORB_SLAM_for_received_map (string vocab_file, string settings_file);
//void runORBSLAMTracking (ORB_SLAM2::System *, sl::Mat, sl::Mat, int, bool isStereo, ofstream* featureFile=NULL);//periodic tracking using ORB SLAM


void saveTrajectoryORBSLAMinSLAMMode (ORB_SLAM2::System * ptrToORBSLAM, const std::string pathToSaveTrajectory);
void saveTrajectoryInMonoCamera (ORB_SLAM2::System * ptrToORBSLAM, const std::string pathToSaveTrajectory);


//ERROR_CODE enableSpatialMap (Camera *, SpatialMappingParameters *);
void initializeORBSlam (string, string, bool);//initialize ORB SLAM system

void shutDownORBSLAM (ORB_SLAM2::System *, std::string, bool, std::string);//Shut down ORB SLAM and record trajectory
//void runZEDSLAM (Camera *);
//void shutDownZEDSLAM (Camera *, bool, String = NULL);

ofstream openFeatureFile (string pathToFile);

void printNumberOfKeyframes (ORB_SLAM2::System *ptrToORBSLAMSystem);

unsigned long int GetNumberOfKeyFrames (ORB_SLAM2::System *);

void validateAllMapPoints (ORB_SLAM2::System *ptrToSystem, int counter);

int GetDescriptorDistance(const cv::Mat &a, const cv::Mat &b);

void InitPaths (cv::FileStorage fSettings, string &pathToSaveMap, string &pathToLoadMap,
                string &pathToSaveTrajectory,
                string &pathToSaveCameraCalib, string &pathToLoadCameraCalib,
                string kittiSequenceNumber, string loadKittiSequenceNumber,
                bool &dynamicObjectRemoval, const bool useBin);

void InitMode (cv::FileStorage fSettings, const int opCode, int &startFrom, int &stopAt, bool &saveMap, bool &reuseMap, bool &useBin);

std::string GetMatType(const cv::Mat& mat);

bool ConvertStringToBool (string inputStr);





#endif //HD_MAP_PROJECT_ZEDINITIALIZATION_H
