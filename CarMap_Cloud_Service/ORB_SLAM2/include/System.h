
/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef SYSTEM_H
#define SYSTEM_H

#include<string>
#include<thread>
#include<opencv2/core/core.hpp>

#include "Tracking.h"
//#include "FrameDrawer.h"
//#include "MapDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
//#include "Viewer.h"
#include "ORBmatcher.h"


#include "BoostArchiver.h"
// for map file io
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include "MapSegmentUploader.h"
#include <boost/asio.hpp>

namespace ORB_SLAM2
{

    class Viewer;
    class FrameDrawer;
    class Map;
    class Tracking;
    class LocalMapping;
    class LoopClosing;
    class MapSegmentUploader;

    class System
    {
    public:
        // Input sensor
        enum eSensor{
            MONOCULAR=0,
            STEREO=1,
            RGBD=2
        };

    public:

        // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
        System (const string &strVocFile, const string &strSettingsFile, const eSensor sensor, Map* map_ptr);
        System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer, bool is_save_map, bool reuseMap, string pathToMapFile, bool runLocalizationMode, const string ipAddress, const int portNumber, bool isUploadMap, string viewerWindow, bool reconstructMap, bool isStitchingMode, bool isDynamicObjectRemoval );
        System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer, bool is_save_map, bool reuseMap, string pathToMapFile, bool runLocalizationMode, const string ipAddress, const int portNumber, bool isUploadMap, string viewerWindow, bool reconstructMap, bool isStitchingMode, bool isDynamicObjectRemoval, bool majorityVoting, bool robustFeatureSearch );
        // Constructor for map received over the network
        System (const string &str_voc_file, const string &str_settings_file);
        //System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true, bool is_save_map_=false);
        // Proccess the given stereo frame. Images must be synchronized and rectified.
        // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
        // Returns the camera pose (empty if tracking fails).
        cv::Mat TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const cv::Mat &imLabel, const cv::Mat &imOverlay, const double &timestamp, ofstream *featureFile = NULL);
        cv::Mat TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const cv::Mat &imLabel, const cv::Mat &imOverlay, const cv::Mat &imDepth, const double &timestamp, ofstream *featureFile = NULL);
        cv::Mat TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp, ofstream *featureFile = NULL);


        cv::Mat TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const cv::Mat &imLabel, const cv::Mat &imOverlay, const cv::Mat &imDP, const cv::Mat &imMN, const cv::Mat &imFDP, const double &timestamp, ofstream *featureFile = NULL);

        cv::Mat TrackKeyFrame (ORB_SLAM2::KeyFrame *kf, bool);

        float GetMapSizeEstimate ();

        vector<KeyFrame*> GetAllKeyFrames ();


        void SemanticSegmentationBenchmark ();

        void RunLocalBundleAdjustmentOnAllNodes();

        bool AreLabelClassesConsistent (unsigned char, unsigned char);
        bool AreLabelsSame (unsigned char, unsigned char);

        // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
        // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
        // Input depthmap: Float (CV_32F).
        // Returns the camera pose (empty if tracking fails).
        cv::Mat TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp);

        // Proccess the given monocular frame
        // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
        // Returns the camera pose (empty if tracking fails).
        cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp);


        void DiffOperationMapPointOriginsTestCase ();

        // This stops local mapping thread (map building) and performs only camera tracking.
        void ActivateLocalizationMode();
        // This resumes local mapping thread and performs SLAM again.
        void DeactivateLocalizationMode();

        // Returns true if there have been a big map change (loop closure, global BA)
        // since last call to this function
        bool MapChanged();

        // Reset the system (clear map)
        void Reset();

        // All threads will be requested to finish.
        // It waits until all threads have finished.
        // This function must be called before saving the trajectory.
        void Shutdown(string);

        // Save camera trajectory in the TUM RGB-D dataset format.
        // Only for stereo and RGB-D. This method does not work for monocular.
        // Call first Shutdown()
        // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
        void SaveTrajectoryTUM(const string &filename);

        // Save keyframe poses in the TUM RGB-D dataset format.
        // This method works for all sensor input.
        // Call first Shutdown()
        // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
        void SaveKeyFrameTrajectoryTUM(const string &filename);

        // Save camera trajectory in the KITTI dataset format.
        // Only for stereo and RGB-D. This method does not work for monocular.
        // Call first Shutdown()
        // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
        void SaveTrajectoryKITTI(const string &filename);
        // Information from most recent processed frame
        // You can call this right after TrackMonocular (or stereo or RGBD)
        int GetTrackingState();
        std::vector<MapPoint*> GetTrackedMapPoints();
        std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

        void SaveMap(const string &filename);

        void SendMapOverNetwork ();

        void ReLocalize ();

        void PrintAllCoVisibilityRelations ();

        unsigned long GetNumberOfKeyframes ();

        void PrintKeyFramesAndMapPoints ();

        void UpdatePositionsOfAllMapPoints ();

        void PrintKeyFrameExistencePoints ();

        void MapReconstruction (const string &cameraSettings, bool reconstruct = true);

        void ReconstructReceivedMap (ORB_SLAM2::Map *map_ptr);

        void IntegrateDiffInMap (ORB_SLAM2::Map* map_ptr);

        unsigned long GetNumberOfMapPoints ();

        unsigned long GetNumberOfDescriptors ();

        void CompareObservationsAndDescriptorList ();

        std::vector <ORB_SLAM2::MapPoint*> GetAllMapPoints ();

        int GetDescriptorDistance (cv::Mat, cv::Mat);

        void PrintDescriptorDistances (void);

        void CopyMapPointIndicies ();

        void ReconstructMapPointObservations (MapPoint *);

        void ValidateMapPointObservationReconstruction ();

        void MapPointKeyFrameRelations (MapPoint *);

        void EnumerateKeyframeIDs (KeyFrame *);

        void EstablishConnectionWithServer ();

        ORBVocabulary* GetOrbVocabulary ();

        void GetMapSize ();

        Map* GetMap ();

        void MajorityVoting ();

        KeyFrameDatabase* GetKeyFrameDatabase ();



        void UpdatePositionInDrawer (cv::Mat);

    public:
        float fx, fy, cx, cy, invfx, invfy, mbf, mThDep;

    private:
        // Save/Load functions

        bool LoadMap(const string &filename, const string &cameraSettings, bool reconstruct);

        cv::Mat getTransformation (cv::Mat, cv::Mat);
        float get3Ddistance (float, float, float);

    private:

        // Map File Path
        std::string pathToMapFile;

        string settFile;

        bool connectionEstablished = false;

        bool stitchingMode;

        // Input sensor
        eSensor mSensor;

        // ORB vocabulary used for place recognition and feature matching.
        ORBVocabulary* mpVocabulary;

        // KeyFrame database for place recognition (relocalization and loop detection).
        KeyFrameDatabase* mpKeyFrameDatabase;

        // Map structure that stores the pointers to all KeyFrames and MapPoints.
        Map* mpMap;

        string mapfile;
        bool is_save_map;

        int map_size;

        const float BYTES_TO_MB = 0.000001;

        // Tracker. It receives a frame and computes the associated camera pose.
        // It also decides when to insert a new keyframe, create some new MapPoints and
        // performs relocalization if tracking fails.
        Tracking* mpTracker;

        // Local Mapper. It manages the local map and performs local bundle adjustment.
        LocalMapping* mpLocalMapper;

        // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
        // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
        LoopClosing* mpLoopCloser;

        // The viewer draws the map and the current camera pose. It uses Pangolin.
//        Viewer* mpViewer;

//        FrameDrawer* mpFrameDrawer;
//        MapDrawer* mpMapDrawer;

        // System threads: Local Mapping, Loop Closing, Viewer.
        // The Tracking thread "lives" in the main execution thread that creates the System object.
        std::thread* mptLocalMapping;
        std::thread* mptLoopClosing;
        std::thread* mptViewer;
        std::thread* uploader;

        MapSegmentUploader* uploaderObject;

        // Reset flag
        std::mutex mMutexReset;
        bool mbReset;

        // Change mode flags
        std::mutex mMutexMode;
        bool mbActivateLocalizationMode;
        bool mbDeactivateLocalizationMode;

        // Tracking state
        int mTrackingState;
        std::vector<MapPoint*> mTrackedMapPoints;
        std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
        std::mutex mMutexState;

        const string serverIpAddress;
        const int serverPortNumber;

        boost::asio::io_service ioService;
        boost::asio::ip::tcp::socket tcpSocket;

        unsigned long int nextMpID;
        unsigned long int nextKpID;
    };

}// namespace ORB_SLAM

#endif // SYSTEM_H
