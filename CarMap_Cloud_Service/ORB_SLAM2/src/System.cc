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

#include "System.h"
#include "Converter.h"
#include <thread>
// #include <pangolin/pangolin.h>
#include <iomanip>
//#include <GlobalDefintions.h>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <unistd.h>
//#include <MapSegmentUploader.h>
#include <boost/asio.hpp>
#include <boost/exception/all.hpp>
#include <exception>
#include <ctime>



using boost::asio::ip::tcp;

static bool has_suffix(const std::string &str, const std::string &suffix) {
    std::size_t index = str.find(suffix, str.size() - suffix.size());
    return (index != std::string::npos);
}

namespace ORB_SLAM2 {

    System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
                   const bool bUseViewer, bool is_save_map_, bool reuseMap, string mapFile, bool runLocalizationMode,
                   const string ipAddress, const int portNumber, bool isUploadMap, string viewerWindow,
                   bool reconstructMap, bool isStitchingMode, bool isDynamicObjectRemoval)

            //Initialization list
            : mSensor(sensor), is_save_map(is_save_map_),
              mbReset(false),
              mbActivateLocalizationMode(false),
              mbDeactivateLocalizationMode(false),
              tcpSocket (ioService),
              serverPortNumber (portNumber),
              serverIpAddress (ipAddress),
              map_size(0),
              stitchingMode (isStitchingMode)
    {

        //Check settings file
        cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
        if (!fsSettings.isOpened()) {
            cerr << "Failed to open settings file at: " << strSettingsFile << endl;
            exit(-1);
        }

        settFile = strSettingsFile;
        //Load vocabulary file
        mpVocabulary = new ORBVocabulary();
        bool bVocLoad = false; // chose loading method based on file extension
        if (has_suffix(strVocFile, ".txt"))
            bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
        else if (has_suffix(strVocFile, ".bin"))
            bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);
        else
            bVocLoad = false;
        if (!bVocLoad) {
            cerr << "Wrong path to vocabulary. " << endl;
            cerr << "Failed to open at: " << strVocFile << endl;
            exit(-1);
        }
        //Load map file
        bool bReuseMap = false;
        //Reuse the map?
        if (reuseMap && mapFile != "")
        {
            pathToMapFile = mapFile;
            LoadMap(pathToMapFile, strSettingsFile, reconstructMap);//Load the map file from the disk
            bReuseMap = true;
        }
        else//if not, then create a new map file
        {
            cout << "Creating new map file" << endl;
            mpKeyFrameDatabase = new KeyFrameDatabase(mpVocabulary);
            mpMap = new Map();
        }
        //Upload map on the fly?
        if (isUploadMap == true)
            EstablishConnectionWithServer();
        //Create Drawers. These are used by the Viewer
//        mpFrameDrawer = new FrameDrawer(mpMap, bReuseMap);
//        mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

        //Initialize the Tracking thread
        //(it will live in the main thread of execution, the one that called this constructor)
        mpTracker = new Tracking(this, mpVocabulary,
                                 mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor, isDynamicObjectRemoval, bReuseMap );

        //Initialize the Local Mapping thread and launch
        mpLocalMapper = new LocalMapping(mpMap, mSensor == MONOCULAR, isStitchingMode, isDynamicObjectRemoval);
        mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);

        //Initialize the Loop Closing thread and launch
        mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor != MONOCULAR);
        mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);


        //Initialize the Viewer thread and launch
/*
        if (bUseViewer) {
            mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker, strSettingsFile, bReuseMap,
//                                  runLocalizationMode, viewerWindow);

//            cout << "Initialized the viewer" << endl;
//            cin.get();
//            cout << "Now running thread in parallel" << endl;
            mptViewer = new thread(&Viewer::Run, mpViewer);
//            cin.get();
//            cout << "Now using set viewer" << endl;
            mpTracker->SetViewer(mpViewer);
//            cout << "It all worked" << endl;
//            cin.get();
        }
*/
        //Set pointers between threads
        mpTracker->SetLocalMapper(mpLocalMapper);
        mpTracker->SetLoopClosing(mpLoopCloser);

        mpLocalMapper->SetTracker(mpTracker);
        mpLocalMapper->SetLoopCloser(mpLoopCloser);

        mpLoopCloser->SetTracker(mpTracker);
        mpLoopCloser->SetLocalMapper(mpLocalMapper);


    }

    System::System (const string &str_voc_file, const string &str_settings_file)
    :
            mSensor(System::STEREO),
            is_save_map(false),
            mbReset(false),
            mbActivateLocalizationMode(false),
            mbDeactivateLocalizationMode(false),
            tcpSocket (ioService),
            serverPortNumber (0),
            serverIpAddress (""),
            map_size(0),
            stitchingMode (false)
    {
        cout << "Constructor for map received over the network ...." << endl;
        //Check settings file
        cv::FileStorage fsSettings(str_settings_file.c_str(), cv::FileStorage::READ);
        if (!fsSettings.isOpened()) {
            cerr << "Failed to open settings file at: " << str_settings_file << endl;
            exit(-1);
        }
        settFile = str_settings_file;

        //Load vocabulary file
        mpVocabulary = new ORBVocabulary();
        bool bVocLoad = false; // chose loading method based on file extension
        if (has_suffix(str_voc_file, ".txt"))
            bVocLoad = mpVocabulary->loadFromTextFile(str_voc_file);
        else if (has_suffix(str_voc_file, ".bin"))
            bVocLoad = mpVocabulary->loadFromBinaryFile(str_voc_file);
        else
            bVocLoad = false;
        if (!bVocLoad) {
            cerr << "Wrong path to vocabulary. " << endl;
            cerr << "Failed to open at: " << str_voc_file << endl;
            exit(-1);
        }

//        mpMap = map_ptr;
//        cout << "Assigned map ptr" << endl;


//        MapReconstruction (str_settings_file, false);
    }


    void System::ReconstructReceivedMap (ORB_SLAM2::Map* map_ptr)
    {
        mpMap = map_ptr;
        cout << "Reconstructing received map segment" << endl;
        MapReconstruction (settFile, false);
    }


    System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
                   const bool bUseViewer, bool is_save_map_, bool reuseMap, string mapFile, bool runLocalizationMode,
                   const string ipAddress, const int portNumber, bool isUploadMap, string viewerWindow,
                   bool reconstructMap, bool isStitchingMode, bool isDynamicObjectRemoval, bool majorityVoting, bool isRobustFeatureSearch)

    //Initialization list
            : mSensor(sensor), is_save_map(is_save_map_),

              mbReset(false),
              mbActivateLocalizationMode(false),
              mbDeactivateLocalizationMode(false),
              tcpSocket (ioService),
              serverPortNumber (portNumber),
              serverIpAddress (ipAddress),
              map_size(0),
              stitchingMode (isStitchingMode)
    {


        //Check settings file
        cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
        if (!fsSettings.isOpened()) {
            cerr << "Failed to open settings file at: " << strSettingsFile << endl;
            exit(-1);
        }

        settFile = strSettingsFile;

        //Load vocabulary file
        mpVocabulary = new ORBVocabulary();
        bool bVocLoad = false; // chose loading method based on file extension
        if (has_suffix(strVocFile, ".txt"))
            bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
        else if (has_suffix(strVocFile, ".bin"))
            bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);
        else
            bVocLoad = false;
        if (!bVocLoad) {
            cerr << "Wrong path to vocabulary. " << endl;
            cerr << "Failed to open at: " << strVocFile << endl;
            exit(-1);
        }


        //Load map file
        bool bReuseMap = false;
        //Reuse the map?
        if (reuseMap && mapFile != "") {
            pathToMapFile = mapFile;
            LoadMap(pathToMapFile, strSettingsFile, reconstructMap);//Load the map file from the disk
            bReuseMap = true;
        } else//if not, then create a new map file
        {
            cout << "Creating new map file" << endl;
            mpKeyFrameDatabase = new KeyFrameDatabase(mpVocabulary);
            mpMap = new Map();
        }


        //Upload map on the fly?
        if (isUploadMap == true)
            EstablishConnectionWithServer();

//        cout << "NO problem till now" << endl;
//        cin.get();


        //Create Drawers. These are used by the Viewer
//        mpFrameDrawer = new FrameDrawer(mpMap, bReuseMap);
//        mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

        //Initialize the Tracking thread
        //(it will live in the main thread of execution, the one that called this constructor)
        mpTracker = new Tracking(this, mpVocabulary,
                                 mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor, isDynamicObjectRemoval, majorityVoting, isRobustFeatureSearch, bReuseMap );

        //Initialize the Local Mapping thread and launch
        mpLocalMapper = new LocalMapping(mpMap, mSensor == MONOCULAR, isStitchingMode,  isDynamicObjectRemoval, majorityVoting);
        mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);

        //Initialize the Loop Closing thread and launch
        mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor != MONOCULAR);
        mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);


//        cout << "No problem till initilaization" << endl;
//        cin.get();
        //Initialize the Viewer thread and launch
        /*
        if (bUseViewer) {
            mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker, strSettingsFile, bReuseMap,
                                  runLocalizationMode, viewerWindow);

//            cout << "Initialized the viewer" << endl;
//            cin.get();
//            cout << "Now running thread in parallel" << endl;
            mptViewer = new thread(&Viewer::Run, mpViewer);
//            cin.get();
//            cout << "Now using set viewer" << endl;
            mpTracker->SetViewer(mpViewer);
//            cout << "It all worked" << endl;
//            cin.get();
        }
         */

//        cout << "Crashing?" << endl;
//        cin.get();
//        cout << "No crash here either" << endl;
        //Set pointers between threads
        mpTracker->SetLocalMapper(mpLocalMapper);
        mpTracker->SetLoopClosing(mpLoopCloser);

        mpLocalMapper->SetTracker(mpTracker);
        mpLocalMapper->SetLoopCloser(mpLoopCloser);

        mpLoopCloser->SetTracker(mpTracker);
        mpLoopCloser->SetLocalMapper(mpLocalMapper);



        //
//        cout << "Skipping local bundle adjustment" << endl;
//        cout << "Waiting for a few more seconds to crash" << endl;
//        sleep (3);
//        cout << "Done waiting" << endl;
//        RunLocalBundleAdjustmentOnAllNodes();
    }




    void System::RunLocalBundleAdjustmentOnAllNodes ()
    {

        cout << "Running local bundle adjustment on all nodes in the map" << endl;
        vector <ORB_SLAM2::KeyFrame*> allKFs = GetAllKeyFrames();

        float totalError = 0;
        int kfCounter = 1;
        for (ORB_SLAM2::KeyFrame* kf : allKFs)
        {

            for (int pKFCounter = 1; pKFCounter < allKFs.size(); pKFCounter++)
            {
                if (allKFs.at(pKFCounter)->mnId == kfCounter)
                {
                    kf = allKFs.at(pKFCounter);
                    cout << "KF for BA = " << kf->mnId << endl;
                    kfCounter++;
                    break;
                }
            }

            cv::Mat prePose = kf->GetPose();
            mpLocalMapper->LocalBA(kf);
            cv::Mat postPose = kf->GetPose();


            cv::Mat diff = getTransformation (prePose, postPose); // Get the difference between the estimated pose and the ground truth
            float error = get3Ddistance (diff.at<float>(0,3), diff.at<float>(1,3), diff.at<float>(2,3));

            totalError += error;

            cout << "Change in position = " << error << endl;


        }

        cout << "Total change in position = " << totalError << endl;

        cout << "Done with local bundle adjustment" << endl;

    }

    cv::Mat System::getTransformation (cv::Mat initialPos, cv::Mat finalPos)
    {
        cv::Mat error(4, 4, CV_32F);
        error.at<float>(0,3) = finalPos.at<float>(0,3) - initialPos.at<float>(0,3);
        error.at<float>(1,3) = finalPos.at<float>(1,3) - initialPos.at<float>(1,3);
        error.at<float>(2,3) = finalPos.at<float>(2,3) - initialPos.at<float>(2,3);

        return error;
    }

    float System::get3Ddistance (float x, float y, float z)
    {
        return sqrt( pow(x,2) + pow (y,2) + pow (z,2));
    }



    void System::UpdatePositionInDrawer (cv:: Mat pose)
    {
//        mpMapDrawer->SetCurrentCameraPose(pose);
        return;

    }

    void System::EstablishConnectionWithServer() {


        cout << "Establishing connection with server" << endl;


        tcpSocket.connect(
                boost::asio::ip::tcp::endpoint(
                        boost::asio::ip::address::from_string( serverIpAddress ),
                        serverPortNumber
                )
        );

        cout << "Connected to server" << endl;
    }

    ORB_SLAM2::Map* System::GetMap ()
    {
        return mpMap;
    }

    cv::Mat System::TrackKeyFrame(ORB_SLAM2::KeyFrame *kf, bool isAnchorPointStitching) {


        if (mSensor != STEREO) {
            cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
            exit(-1);
        }

        /////////////////////
        //GUI Functions
        ///////////////////
        // Check mode change
        {
            unique_lock<mutex> lock(mMutexMode);
            if (mbActivateLocalizationMode) {
                mpLocalMapper->RequestStop();

                // Wait until Local Mapping has effectively stopped
                while (!mpLocalMapper->isStopped()) {
                    std::this_thread::sleep_for(std::chrono::microseconds(1000));
                }

                mpTracker->InformOnlyTracking(true);
                mbActivateLocalizationMode = false;
            }
            if (mbDeactivateLocalizationMode) {
                mpTracker->InformOnlyTracking(false);
                mpLocalMapper->Release();
                mbDeactivateLocalizationMode = false;
            }
        }
        // Check reset
        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbReset) {
                mpTracker->Reset();
                mbReset = false;
            }
        }


       return mpTracker->GrabStereoKeyFrame(kf, isAnchorPointStitching);
    }

    ORBVocabulary* System::GetOrbVocabulary (void)
    {
        return mpVocabulary;
    }

    KeyFrameDatabase* System::GetKeyFrameDatabase (void)
    {
        return mpKeyFrameDatabase;
    }

    cv::Mat
    System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp, ofstream *featureFile) {
        if (mSensor != STEREO) {
            cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
            exit(-1);
        }



        /////////////////////
        //GUI Functions
        ///////////////////
        // Check mode change
        {
            unique_lock<mutex> lock(mMutexMode);
            if (mbActivateLocalizationMode) {
                mpLocalMapper->RequestStop();

                // Wait until Local Mapping has effectively stopped
                while (!mpLocalMapper->isStopped()) {
                    std::this_thread::sleep_for(std::chrono::microseconds(1000));
                }

                mpTracker->InformOnlyTracking(true);
                mbActivateLocalizationMode = false;
            }
            if (mbDeactivateLocalizationMode) {
                mpTracker->InformOnlyTracking(false);
                mpLocalMapper->Release();
                mbDeactivateLocalizationMode = false;
            }
        }

        // Check reset
        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbReset) {
                mpTracker->Reset();
                mbReset = false;
            }
        }










//        if (FEATURE_DEBUG_MODE)
//            cout << "TrackStereo:GrabImageStereo()" << endl;

        cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft, imRight, timestamp, featureFile);

        unique_lock<mutex> lock2(mMutexState);
        mTrackingState = mpTracker->mState;
        mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
        mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
        return Tcw;
    }


    cv::Mat
    System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const cv::Mat &imLabel, const cv::Mat &imOverlay, const cv::Mat &imDepth, const double &timestamp, ofstream *featureFile) {
        if (mSensor != STEREO) {
            cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
            exit(-1);
        }

        /////////////////////
        //GUI Functions
        ///////////////////
        // Check mode change
        {
            unique_lock<mutex> lock(mMutexMode);
            if (mbActivateLocalizationMode) {
                mpLocalMapper->RequestStop();

                // Wait until Local Mapping has effectively stopped
                while (!mpLocalMapper->isStopped()) {
                    std::this_thread::sleep_for(std::chrono::microseconds(1000));
                }

                mpTracker->InformOnlyTracking(true);
                mbActivateLocalizationMode = false;
            }
            if (mbDeactivateLocalizationMode) {
                mpTracker->InformOnlyTracking(false);
                mpLocalMapper->Release();
                mbDeactivateLocalizationMode = false;
            }
        }

        // Check reset
        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbReset) {
                mpTracker->Reset();
                mbReset = false;
            }
        }

        cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft, imRight, imLabel, imOverlay , imDepth, timestamp, featureFile);

        unique_lock<mutex> lock2(mMutexState);
        mTrackingState = mpTracker->mState;
        mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
        mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
        return Tcw;
    }


    cv::Mat
    System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight,
            const cv::Mat &imLabel, const cv::Mat &imOverlay,
            const cv::Mat &imDP, const cv::Mat &imMN, const cv::Mat &imFDP,
            const double &timestamp, ofstream *featureFile) {
        if (mSensor != STEREO) {
            cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
            exit(-1);
        }

        /////////////////////
        //GUI Functions
        ///////////////////
        // Check mode change
        {
            unique_lock<mutex> lock(mMutexMode);
            if (mbActivateLocalizationMode) {
                mpLocalMapper->RequestStop();

                // Wait until Local Mapping has effectively stopped
                while (!mpLocalMapper->isStopped()) {
                    std::this_thread::sleep_for(std::chrono::microseconds(1000));
                }

                mpTracker->InformOnlyTracking(true);
                mbActivateLocalizationMode = false;
            }
            if (mbDeactivateLocalizationMode) {
                mpTracker->InformOnlyTracking(false);
                mpLocalMapper->Release();
                mbDeactivateLocalizationMode = false;
            }
        }

        // Check reset
        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbReset) {
                mpTracker->Reset();
                mbReset = false;
            }
        }

        cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft, imRight, imLabel, imOverlay , imDP, imMN, imFDP, timestamp, featureFile);

        unique_lock<mutex> lock2(mMutexState);
        mTrackingState = mpTracker->mState;
        mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
        mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
        return Tcw;
    }



    cv::Mat
    System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const cv::Mat &imLabel, const cv::Mat &imOverlay, const double &timestamp, ofstream *featureFile) {
        if (mSensor != STEREO) {
            cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
            exit(-1);
        }



        /////////////////////
        //GUI Functions
        ///////////////////
        // Check mode change
        {
            unique_lock<mutex> lock(mMutexMode);
            if (mbActivateLocalizationMode) {
                mpLocalMapper->RequestStop();

                // Wait until Local Mapping has effectively stopped
                while (!mpLocalMapper->isStopped()) {
                    std::this_thread::sleep_for(std::chrono::microseconds(1000));
                }

                mpTracker->InformOnlyTracking(true);
                mbActivateLocalizationMode = false;
            }
            if (mbDeactivateLocalizationMode) {
                mpTracker->InformOnlyTracking(false);
                mpLocalMapper->Release();
                mbDeactivateLocalizationMode = false;
            }
        }

        // Check reset
        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbReset) {
                mpTracker->Reset();
                mbReset = false;
            }
        }

        cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft, imRight, imLabel, imOverlay , timestamp, featureFile);

        unique_lock<mutex> lock2(mMutexState);
        mTrackingState = mpTracker->mState;
        mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
        mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
        return Tcw;
    }

    cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp) {
        if (mSensor != RGBD) {
            cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
            exit(-1);
        }

        // Check mode change
        {
            unique_lock<mutex> lock(mMutexMode);
            if (mbActivateLocalizationMode) {
                mpLocalMapper->RequestStop();

                // Wait until Local Mapping has effectively stopped
                while (!mpLocalMapper->isStopped()) {
                    std::this_thread::sleep_for(std::chrono::microseconds(1000));
                }

                mpTracker->InformOnlyTracking(true);
                mbActivateLocalizationMode = false;
            }
            if (mbDeactivateLocalizationMode) {
                mpTracker->InformOnlyTracking(false);
                mpLocalMapper->Release();
                mbDeactivateLocalizationMode = false;
            }
        }

        // Check reset
        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbReset) {
                mpTracker->Reset();
                mbReset = false;
            }
        }

        cv::Mat Tcw = mpTracker->GrabImageRGBD(im, depthmap, timestamp);

        unique_lock<mutex> lock2(mMutexState);
        mTrackingState = mpTracker->mState;
        mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
        mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
        return Tcw;
    }

    cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp) {
        if (mSensor != MONOCULAR) {
            cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
            exit(-1);
        }

        // Check mode change
        {
            unique_lock<mutex> lock(mMutexMode);
            if (mbActivateLocalizationMode) {
                mpLocalMapper->RequestStop();

                // Wait until Local Mapping has effectively stopped
                while (!mpLocalMapper->isStopped()) {
                    std::this_thread::sleep_for(std::chrono::microseconds(1000));
                }

                mpTracker->InformOnlyTracking(true);
                mbActivateLocalizationMode = false;
            }
            if (mbDeactivateLocalizationMode) {
                mpTracker->InformOnlyTracking(false);
                mpLocalMapper->Release();
                mbDeactivateLocalizationMode = false;
            }
        }

        // Check reset
        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbReset) {
                mpTracker->Reset();
                mbReset = false;
            }
        }

        cv::Mat Tcw = mpTracker->GrabImageMonocular(im, timestamp);

        unique_lock<mutex> lock2(mMutexState);
        mTrackingState = mpTracker->mState;
        mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
        mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

        return Tcw;
    }

    void System::ActivateLocalizationMode() {
        unique_lock<mutex> lock(mMutexMode);
        mbActivateLocalizationMode = true;
    }

    void System::DeactivateLocalizationMode() {
        unique_lock<mutex> lock(mMutexMode);
        mbDeactivateLocalizationMode = true;
    }

    bool System::MapChanged() {
        static int n = 0;
        int curn = mpMap->GetLastBigChangeIdx();
        if (n < curn) {
            n = curn;
            return true;
        } else
            return false;
    }

    void System::PrintAllCoVisibilityRelations ()
    {
        vector <KeyFrame*> allKfs = GetAllKeyFrames();

        cout << "KF ------ isFirstConnection ------ parent ------ children" << endl;

        for (KeyFrame *kf : allKfs)
        {
            cout << endl;
            KeyFrame *pKF = kf->GetParent();
            cout << "KF [ " << kf->mnId << " ]";

            if (kf->GetIsFirstConnection())
                cout << " ------ TRUE";
            else
                cout << " ------ FALSE";

            cout << " ------ PARENT = ";
            if (pKF == NULL)
                    cout << " NULL";
            else
                cout << kf->GetParent()->mnId;

            set <KeyFrame*> children = kf->GetChilds();

            cout << " ------  Children = ";
            for (KeyFrame *cKF : children)
            {
                cout << cKF->GetmnId() << ", ";
            }

        }

        cout << endl;
    }

    vector<KeyFrame*> System::GetAllKeyFrames ()
    {
        return mpMap->GetAllKeyFrames();
    }

    void System::Reset() {
        unique_lock<mutex> lock(mMutexReset);
        mbReset = true;
    }

    void System::ReLocalize() {
//        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();


        //              size_t start = time(NULL);

        bool localize = mpTracker->Relocalization();
//        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

//        if (localize)
//            cout << "Localized" << endl;
//        else
//            cout << "Failed" << endl;
//        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << " microseconds" << std::endl;
//        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << " nanoseconds" << std::endl;

    }

    bool System::AreLabelClassesConsistent (unsigned char labelOne, unsigned char labelTwo)
    {
        if ( SemanticSegmentor::IsStaticObject(labelOne) && SemanticSegmentor::IsStaticObject(labelTwo) )
            return true;
        else if ( SemanticSegmentor::IsDynamicObject(labelOne) && SemanticSegmentor::IsDynamicObject(labelTwo) )
            return true;
        else
            return false;
    }

    bool System::AreLabelsSame (unsigned char labelOne, unsigned char labelTwo)
    {
        if (labelOne == labelTwo)
            return true;
        else
            return false;
    }

    void System::MajorityVoting () {
        vector<ORB_SLAM2::KeyFrame *> allKfs = mpMap->GetAllKeyFrames();
        vector<ORB_SLAM2::MapPoint *> mps = mpMap->GetAllMapPoints();
        vector<unsigned int> mpIds;
        for (KeyFrame *kf : allKfs) {
            if (kf->isLoadedFromDisk == true)
                continue;

            if (kf->AreMpsLabled() == false) {
                cout << "We might need to do labeling here for keyframe = " << kf->GetmnId() << endl;
            }

            int numberOfFeatures = kf->N;
            for (int featureCounter = 0; featureCounter < numberOfFeatures; featureCounter++) {
                MapPoint *pMP = kf->mvpMapPoints.at(featureCounter);
                if (pMP == NULL)
                    continue;
                else if (pMP->isBad())
                    continue;
                // Majority voting
                if (std::count(mpIds.begin(), mpIds.end(), pMP->GetmnId())) {
                    continue;
                }
                mpIds.push_back(pMP->GetmnId());
                pMP->AssignConsensusLabel();
            }
        }

        cout << "Done with majority voting" << endl;
    }

    void System::SemanticSegmentationBenchmark ()
    {
        vector <ORB_SLAM2::KeyFrame*> allKfs = mpMap->GetAllKeyFrames();
        unsigned int dpCorrect, mnCorrect, fdpCorrect, majorityVoting;
        dpCorrect = 0;
        mnCorrect = 0;
        fdpCorrect = 0;
        majorityVoting = 0;

        unsigned int static_static_error = 0;
        unsigned int static_dynamic_error = 0;
        unsigned int dynamic_dynamic_error = 0;

        unsigned int DPcorrect = 0;
        unsigned int MNcorrect = 0;
        unsigned int FDPcorrect = 0;
        unsigned int MajorityVoting = 0;

        unsigned int unlabeled = 0;
        unsigned int labeled = 0;

        unsigned int totalMps = 0;

        vector <ORB_SLAM2::MapPoint*> mps = mpMap->GetAllMapPoints ();
        const int MpsBeforeRemoval = mps.size();
        cout << "Number of map-points in total = " << mps.size() << endl;



        vector <unsigned int> mpIds;
        for (KeyFrame * kf : allKfs)
        {
            if (kf->isLoadedFromDisk == true)
                continue;

            if (kf->AreMpsLabled() == false)
            {
                cout << "We might need to do labeling here for keyframe = " << kf->GetmnId() << endl;
            }

            int numberOfFeatures = kf->N;
            for ( int featureCounter = 0; featureCounter < numberOfFeatures; featureCounter++ )
            {
                MapPoint *pMP = kf->mvpMapPoints.at(featureCounter);
                if (pMP == NULL)
                    continue;
                else if (pMP->isBad())
                    continue;

                // Majority voting

                if ( std::count ( mpIds.begin(), mpIds.end(), pMP->GetmnId() ) )
                {
                    continue;
                }

                mpIds.push_back (pMP->GetmnId());
                pMP->AssignConsensusLabel();

                if (kf->mvGTLabels [featureCounter] == UNDETECTED_CODE)
                    continue;


                // check DP correct
                if ( AreLabelsSame ( kf->mvGTLabels [featureCounter], kf->mvDPLabels[featureCounter] ) )
                    DPcorrect++;
                // check mnCorrect
                if ( AreLabelsSame ( kf->mvGTLabels [featureCounter], kf->mvMNLabels[featureCounter] ) )
                    MNcorrect++;
                // check fdpCorrect
                if ( AreLabelsSame ( kf->mvGTLabels [featureCounter], kf->mvFDPLabels[featureCounter] ) )
                    FDPcorrect++;


                // check DP correct
                if ( AreLabelClassesConsistent ( kf->mvGTLabels [featureCounter], kf->mvDPLabels[featureCounter] ) )
                    dpCorrect++;
                // check mnCorrect
                if ( AreLabelClassesConsistent ( kf->mvGTLabels [featureCounter], kf->mvMNLabels[featureCounter] ) )
                    mnCorrect++;
                // check fdpCorrect
                if ( AreLabelClassesConsistent ( kf->mvGTLabels [featureCounter], kf->mvFDPLabels[featureCounter] ) )
                    fdpCorrect++;


                if (pMP->GetSemanticLabel() == 20 || pMP->GetSemanticLabel() == 255) {
                    unlabeled++;
                    totalMps++;
                    continue;
                }
                else
                    labeled++;




                // majorityVoting
                if ( AreLabelsSame ( kf->mvGTLabels [featureCounter], pMP->GetSemanticLabel() ) )
                    MajorityVoting++;
                else // the classification was erroneous
                {
                    unsigned char gtLabel = kf->mvGTLabels [featureCounter];
                    unsigned char ptLabel = pMP->GetSemanticLabel();
                    // check if the error was from labels of the same class
                    if ( AreLabelClassesConsistent ( gtLabel, ptLabel  ) )
                    {
                        // Now check if the error was at the boundary of static static objects
                        if ( SemanticSegmentor::IsDynamicObject ( ptLabel ) )
                            dynamic_dynamic_error++;
                        else
                            static_static_error++;

                    }
                    else
                    {
                        static_dynamic_error++;
                    }

                }




                // majorityVoting
                if ( AreLabelClassesConsistent ( kf->mvGTLabels [featureCounter], pMP->GetSemanticLabel() ) ) {
                    majorityVoting++;
                    cout << "YES! " << SemanticSegmentor::GetObjectName(kf->mvGTLabels[featureCounter]) << " = " <<
                    SemanticSegmentor::GetObjectName(pMP->GetSemanticLabel()) << endl;

                }
//                if ( AreLabelClassesConsistent ( kf->mvGTLabels [featureCounter], kf->mvLabels[featureCounter] ) )
                else
                {
                    cout << "NO! " << SemanticSegmentor::GetObjectName(kf->mvGTLabels[featureCounter]) << " = " <<
                         SemanticSegmentor::GetObjectName(pMP->GetSemanticLabel()) << endl;
                }



                totalMps++;
            }
        }

        cout << "Classification of objects classes (dynamic Vs. static)" << endl;
        cout << "dp correct = " << dpCorrect << " / " << totalMps << " = " << (dpCorrect * 100.0 / totalMps * 1.0) << endl;
        cout << "mn correct = " << mnCorrect << " / " << totalMps << " = " << (mnCorrect * 100.0 / totalMps * 1.0) << endl;
        cout << "fdp correct = " << fdpCorrect << " / " << totalMps << " = " << (fdpCorrect * 100.0 / totalMps * 1.0) << endl;
        cout << "current scheme correct = " << majorityVoting << " / " << totalMps << " = " << (majorityVoting * 100.0 / totalMps * 1.0) << endl;

        cout << endl;
        cout << "Classification of objects (different object types)" << endl;
        cout << "DP correct = " << DPcorrect << " / " << totalMps << " = " << (DPcorrect * 100.0 / totalMps * 1.0) << endl;
        cout << "MN correct = " << MNcorrect << " / " << totalMps << " = " << (MNcorrect * 100.0 / totalMps * 1.0) << endl;
        cout << "FDP correct = " << FDPcorrect << " / " << totalMps << " = " << (FDPcorrect * 100.0 / totalMps * 1.0) << endl;
        cout << "Current scheme correct = " << MajorityVoting << " / " << totalMps << " = " << (MajorityVoting * 100.0 / totalMps * 1.0) << endl;


        cout << "Number of total map-points before removal = " << MpsBeforeRemoval << endl;
        vector <MapPoint*> allMps = mpMap->GetAllMapPoints();
        cout << "Number of map-points after removal = " << allMps.size() << endl;
        cout << "Number of dynamic objects removed = " << MpsBeforeRemoval - allMps.size() << endl;


        cout << "Number of map-points labeled = " << labeled << endl;
        cout << "Number of unlabeled = " << unlabeled << endl;


        cout << endl << endl;
        cout << "Break down of errors" << endl;

        int allErrors = dynamic_dynamic_error + static_static_error + static_dynamic_error;

        cout << "Dynamic-dynamic error = " << dynamic_dynamic_error << " / " << allErrors << " = "
        << (dynamic_dynamic_error * 100.0) / allErrors << " % " << endl;

        cout << "Static-static error = " << static_static_error << " / " << allErrors << " = "
             << (static_static_error * 100.0) / allErrors << " % " << endl;

        cout << "Static-dynamic error = " << static_dynamic_error << " / " << allErrors << " = "
             << (static_dynamic_error * 100.0) / allErrors << " % " << endl;

    }

    void System::Shutdown(string pathToSaveMapFile) {

//        SemanticSegmentationBenchmark();
//        MajorityVoting ();



        sleep (2);

        mpLocalMapper->RequestFinish();

        mpLoopCloser->RequestFinish();
        /*
        if (mpViewer) {
            mpViewer->RequestFinish();
            while (!mpViewer->isFinished()) {
                std::this_thread::sleep_for(std::chrono::microseconds(5000));
            }
        }
         */

        // Wait until all thread have effectively stopped
        while (!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA()) {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }

        if (mpLocalMapper->isFinished() == false)
            cerr << "Mapping was not finished" << endl;
        if (mpLoopCloser->isFinished() == false)
            cerr << "Loop closing was not finished" << endl;



        cout << "Closing down the tracking thread" << endl;
        if (mpTracker->GetThreadStatus())
            cerr << "Tracking was still running?" << endl;


        //if (mpViewer)
        //    pangolin::BindToContext("ORB-SLAM2: Map Viewer");
        if (is_save_map)//Save the map?
            SaveMap(pathToSaveMapFile);
        else;//no need to save the map

    }

    void System::SaveTrajectoryTUM(const string &filename) {
        cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
        if (mSensor == MONOCULAR) {
            cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
            return;
        }

        vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        cv::Mat Two = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
        // We need to get first the keyframe pose and then concatenate the relative transformation.
        // Frames not localized (tracking failure) are not saved.

        // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
        // which is true when tracking failed (lbL).
        list<ORB_SLAM2::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
        list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        list<bool>::iterator lbL = mpTracker->mlbLost.begin();
        for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
                     lend = mpTracker->mlRelativeFramePoses.end(); lit != lend; lit++, lRit++, lT++, lbL++) {
            if (*lbL)
                continue;

            KeyFrame *pKF = *lRit;

            cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

            // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
            while (pKF->isBad()) {
                Trw = Trw * pKF->mTcp;
                pKF = pKF->GetParent();
            }

            Trw = Trw * pKF->GetPose() * Two;

            cv::Mat Tcw = (*lit) * Trw;
            cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
            cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

            vector<float> q = Converter::toQuaternion(Rwc);

            f << setprecision(6) << *lT << " " << setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " "
              << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
        }
        f.close();
        cout << endl << "trajectory saved!" << endl;


    }

    void System::SaveKeyFrameTrajectoryTUM(const string &filename) {
        cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

        vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        //cv::Mat Two = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        for (size_t i = 0; i < vpKFs.size(); i++) {
            KeyFrame *pKF = vpKFs[i];

            // pKF->SetPose(pKF->GetPose()*Two);

            if (pKF->isBad())
                continue;

            cv::Mat R = pKF->GetRotation().t();
            vector<float> q = Converter::toQuaternion(R);
            cv::Mat t = pKF->GetCameraCenter();
            f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1)
              << " " << t.at<float>(2)
              << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

        }

        f.close();
        cout << endl << "trajectory saved!" << endl;
    }

    void System::SaveTrajectoryKITTI(const string &filename) {

        cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
        if (mSensor == MONOCULAR) {
            cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
            return;
        }

        vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);


        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        cv::Mat Two = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(filename.c_str());
        f << fixed;



        // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
        // We need to get first the keyframe pose and then concatenate the relative transformation.
        // Frames not localized (tracking failure) are not saved.

        // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
        // which is true when tracking failed (lbL).
        list<ORB_SLAM2::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
        list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        list<bool>::iterator lbL = mpTracker->mlbLost.begin();

        long int counter = 0;
        bool isFirstElement = true;
        for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(), lend = mpTracker->mlRelativeFramePoses.end();
             lit != lend; lit++, lRit++, lbL++, lT++, counter++) {
            string localizationStatus = *lbL ? "lost" : "tracked";
            ORB_SLAM2::KeyFrame *pKF = *lRit;
            cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

            if (pKF != NULL) {
                while (pKF->isBad()) {
                    Trw = Trw * pKF->mTcp;
                    pKF = pKF->GetParent();

                }

                Trw = Trw * pKF->GetPose() * Two;

                cv::Mat Tcw = (*lit) * Trw;
                cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
                cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);




//                f << setprecision(0) << *lT << " : ";
//                f << setprecision(9) << Rwc.at<float>(0, 0) << " " << Rwc.at<float>(0, 1) << " " << Rwc.at<float>(0, 2)
//                  << " " << twc.at<float>(0) << " " <<
//                  Rwc.at<float>(1, 0) << " " << Rwc.at<float>(1, 1) << " " << Rwc.at<float>(1, 2) << " "
//                  << twc.at<float>(1) << " " <<
//                  Rwc.at<float>(2, 0) << " " << Rwc.at<float>(2, 1) << " " << Rwc.at<float>(2, 2) << " "
//                  << twc.at<float>(2) << endl;
//
//                f <<  *lT << " " << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << endl;
                f << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << endl;
//
//                    f << "0 0 0 " << twc.at<float>(0) << " 0 0 0 " << twc.at<float>(1) << " 0 0 0 " << twc.at<float>(2) << endl;
            } else
                f << localizationStatus << endl;

        }
        f.close();
        cout << endl << "SLAM Trajectory saved with " << counter << " frames!" << endl;



        /*
            cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
            if(mSensor==MONOCULAR)
            {
                cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
                return;
            }

            vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
            sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

            // Transform all keyframes so that the first keyframe is at the origin.
            // After a loop closure the first keyframe might not be at the origin.
            cv::Mat Two = vpKFs[0]->GetPoseInverse();

            ofstream f;
            f.open(filename.c_str());
            f << fixed;

            // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
            // We need to get first the keyframe pose and then concatenate the relative transformation.
            // Frames not localized (tracking failure) are not saved.

            // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
            // which is true when tracking failed (lbL).
            list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
            list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
            for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
            {
                ORB_SLAM2::KeyFrame* pKF = *lRit;

                cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

                if (pKF != NULL)
                while(pKF->isBad())
                {
                    //  cout << "bad parent" << endl;
                    Trw = Trw*pKF->mTcp;
                    pKF = pKF->GetParent();
                }

                Trw = Trw*pKF->GetPose()*Two;

                cv::Mat Tcw = (*lit)*Trw;
                cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
                cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

                f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
                  Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
                  Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
            }
            f.close();
            cout << endl << "trajectory saved!" << endl;
    */
    }

    int System::GetTrackingState() {
        unique_lock<mutex> lock(mMutexState);
        return mTrackingState;
    }

    vector<MapPoint *> System::GetTrackedMapPoints() {
        unique_lock<mutex> lock(mMutexState);
        return mTrackedMapPoints;
    }

    vector<cv::KeyPoint> System::GetTrackedKeyPointsUn() {
        unique_lock<mutex> lock(mMutexState);
        return mTrackedKeyPointsUn;
    }

    float System::GetMapSizeEstimate ()
    {
        unsigned int long keyFrameSize = 0;
        unsigned int long mapPointSize = 0;
        string mapFile;
        //Populate keyframes
        for (KeyFrame *kf : mpMap->GetAllKeyFrames())
        {
            string keyframe;
            keyframe = to_string(kf->mnId);

            //Populate pose array
            for (int row = 0; row < 4; row++)
            {
                for (int col = 0; col < 4; col++)
                    keyframe += to_string(kf->GetPose().at<float>(row,col));
            }

            //Populate mvpMapPoints
            for (MapPoint *kfMp : kf->GetMapPoints())
            {
                if (kfMp != NULL)
                    if (kfMp->isBad() == false)
                        keyframe += to_string(kfMp->mnId);
            }

            //Co-visibility graph
            if (kf->GetParent() != NULL)
                if (kf->GetParent()->isBad() == false)
                    keyframe += to_string(kf->GetParent()->mnId);
            for (KeyFrame *child : kf->GetChilds())
                keyframe += to_string (child->mnId);



//            cout << "Size of keyframe " << kf->mnId << " = " << keyframe.size()  << endl;
            mapFile += keyframe;
            keyFrameSize += keyframe.size();


        }


        //Populate map points
        for (MapPoint *mp : mpMap->GetAllMapPoints())
        {
            string mapPoint;

            //Store ID
            mapPoint = mp->mnId;
            //Store world position
            mapPoint += to_string (mp->GetWorldPos().at<float>(0,1))
                        + to_string (mp->GetWorldPos().at<float>(0,2))
                        + to_string (mp->GetWorldPos().at<float>(0,3));

            for (int counter = 0; counter < 32; counter++)
                mapPoint += to_string(mp->GetDescriptor().at<int>(counter));


//            cout << "Size of map point " << mp->mnId << " = " << mapPoint.size() << endl;

            mapFile += mapPoint;
            mapPointSize += mapPoint.size();
        }



//        cout << "Total size of map = " << mapFile.size() * 0.000001 << " megabytes" << endl;
        return mapFile.size() * 0.000001;

    }

    void System::GetMapSize ()
    {
        mpLocalMapper->RequestStop();

        // Wait until Local Mapping has effectively stopped
        while (!mpLocalMapper->isStopped()) {
            std::this_thread::sleep_for(std::chrono::microseconds(1000));
        }

        boost::asio::streambuf buf;
        std::ostream os( &buf );
        boost::archive::binary_oarchive ar( os );

        try{

            ar & mpMap;

        }


        catch (boost::system::system_error e)
        {
            std::cerr << boost::diagnostic_information(e);
        }

        std::time_t result = std::time(nullptr);//get current time

        std::cout << result << ", " << buf.size() * BYTES_TO_MB << ", " << ((map_size + buf.size()) * BYTES_TO_MB) << endl;


        mpLocalMapper->Release();


    }

    void System::SendMapOverNetwork ()
    {
//        sleep (0);

            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while (!mpLocalMapper->isStopped()) {
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
            }
        boost::asio::streambuf buf;
        std::ostream os( &buf );
        boost::archive::binary_oarchive ar( os );

        try{

        ar & mpMap;

        }


        catch (boost::system::system_error e)
        {
            std::cerr << boost::diagnostic_information(e);
        }


        const size_t header = buf.size();

        // send header and buffer using scatter
        std::vector<boost::asio::const_buffer> buffers;
        buffers.push_back( boost::asio::buffer(&header, sizeof(header)) );
        buffers.push_back( buf.data() );

        size_t dataSent = 0;
        try {
            const size_t rc = boost::asio::write(
                    tcpSocket,
                    buffers
            );

            dataSent = rc;


        }
        catch (boost::system::system_error e)
        {
            std::cerr << boost::diagnostic_information(e);
        }


        std::time_t result = std::time(nullptr);//get current time
        map_size += dataSent;

        std::cout << result << ", " << dataSent * BYTES_TO_MB << ", " << map_size * BYTES_TO_MB << endl;


        Reset();

        mpLocalMapper->Release();


/*
        const unsigned int number_of_keyframes = mpMap->GetAllKeyFrames().size();
        const unsigned int number_of_mappoints = mpMap->GetAllMapPoints().size();

        string mapFile = "";

        unsigned int long keyFrameSize = 0;
        unsigned int long mapPointSize = 0;
        //Populate keyframes
        for (KeyFrame *kf : mpMap->GetAllKeyFrames())
        {
            string keyframe;
            keyframe = to_string(kf->mnId);

            //Populate pose array
            for (int row = 0; row < 4; row++)
            {
                for (int col = 0; col < 4; col++)
                    keyframe += to_string(kf->GetPose().at<float>(row,col));
            }

            //Populate mvpMapPoints
            for (MapPoint *kfMp : kf->GetMapPoints())
            {
                if (kfMp != NULL)
                    if (kfMp->isBad() == false)
                        keyframe += to_string(kfMp->mnId);
            }

            //Co-visibility graph
            if (kf->GetParent() != NULL)
                if (kf->GetParent()->isBad() == false)
                    keyframe += to_string(kf->GetParent()->mnId);
            for (KeyFrame *child : kf->GetChilds())
                keyframe += to_string (child->mnId);



//            cout << "Size of keyframe " << kf->mnId << " = " << keyframe.size()  << endl;
            mapFile += keyframe;
            keyFrameSize += keyframe.size();
        }


        //Populate map points
        for (MapPoint *mp : mpMap->GetAllMapPoints())
        {
            string mapPoint;

            //Store ID
            mapPoint = mp->mnId;
            //Store world position
            mapPoint += to_string (mp->GetWorldPos().at<float>(0,1))
                        + to_string (mp->GetWorldPos().at<float>(0,2))
                        + to_string (mp->GetWorldPos().at<float>(0,3));

            for (int counter = 0; counter < 32; counter++)
                mapPoint += to_string(mp->GetDescriptor().at<int>(counter));


//            cout << "Size of map point " << mp->mnId << " = " << mapPoint.size() << endl;

            mapFile += mapPoint;
            mapPointSize += mapPoint.size();
        }



        cout << "Total size of map = " << mapFile.size() * 0.000001 << " megabytes" << endl;
        cout << "Average size of keyframe = " << keyFrameSize * 1.0 / mpMap->GetAllKeyFrames().size() << endl;
        cout << "Average size of map point = " << mapPointSize * 1.0 / mpMap->GetAllMapPoints().size() << endl;


//        uploaderObject->UpdateFile(mapFile, mapFile.size());
//        uploaderObject->DataAvailable();


//        while (uploaderObject->IsDataAvailable() == true);
        cout << "Upload done" << endl;
        cout << "Resetting map" << endl;
        Reset();

*/
//        strcpy (mapFileStrChild, mapFile.c_str());//make it a c string
//        pipe (pip);


        /*
//        pid = fork ();
        if (pid == 0) //Child process
        {
            cout << "Child process is writing" << endl;
            cout << "Sending " << mapFile.size() + 1 << endl;
//            write (pip [1], &mapFileStrChild, mapFile.size() + 1);
            write (pip [1], "ABC", 4);
            return;
        }

        else {
            cout << "Parent process is reading" << endl;
//            read (pip[0], mapFileStrParent, mapFile.size() + 1);
            read (pip[0], mapFileStrParent, 4);
            cout << "Parent: " << mapFileStrParent << endl;
            uploader.Upload();
        }

         */




//        cin.get();


        /*

        cout << "Saving Mapfile: " << mapfile << std::flush;
        if (has_suffix(filename, ".txt") == true) {
            cout << "Saving mapfile in txt format" << endl;


            //Get next IDs for keyframe and map points
            KeyFrame *kf = mpMap->GetAllKeyFrames().at(0);
            nextMpID = GetAllMapPoints().at(0)->nNextId;
            nextKpID = kf->nNextId;

            //Get camera intrinsic properties of keyframes
            fx = kf->fx;
            fy = kf->fy;
            cx = kf->cx;
            cy = kf->cy;
            mbf = kf->mbf;
            mThDep = kf->mThDepth;

            //Copy map point observation indices to map point observation structure
            CopyMapPointIndicies ();


            cout << "Saving map file in bin format" << endl;
            cout << "Number of keyframes = " << GetNumberOfKeyframes() << endl;
            cout << "Number of map points = " << GetNumberOfMapPoints() << endl;
            cout << "Number of descriptors = " << GetNumberOfDescriptors() << endl;
            boost::archive::text_oarchive oa(out, boost::archive::no_header);




            string textHeader = "ORB SLAM Map File\n";
            oa << textHeader;


            //Serialize
            string mapFile = "\nmapFile\n";
            oa << mapFile;
            oa << mpMap;
            string smallVariables = "\nsmallVariables\n";
            oa << smallVariables;
            oa << nextMpID;
            oa << nextKpID;
            oa << fx;
            oa << fy;
            oa << cx;
            oa << cy;
            oa << mbf;
            oa << mThDep;

            cout << " ...done" << std::endl;
            out.close();


        } else {

            //Get next IDs for keyframe and map points
            KeyFrame *kf = mpMap->GetAllKeyFrames().at(0);
            nextMpID = GetAllMapPoints().at(0)->nNextId;
            nextKpID = kf->nNextId;

            //Get camera intrinsic properties of keyframes
            fx = kf->fx;
            fy = kf->fy;
            cx = kf->cx;
            cy = kf->cy;
            mbf = kf->mbf;
            mThDep = kf->mThDepth;

            //Copy map point observation indicies to map point observation structure
            CopyMapPointIndicies ();


            cout << "Saving map file in bin format" << endl;
            cout << "Number of keyframes = " << GetNumberOfKeyframes() << endl;
            cout << "Number of map points = " << GetNumberOfMapPoints() << endl;
            cout << "Number of descriptors = " << GetNumberOfDescriptors() << endl;
            boost::archive::binary_oarchive oa(out, boost::archive::no_header);




            //Serialize
            oa << mpMap;
            oa << nextMpID;
            oa << nextKpID;
            oa << fx;
            oa << fy;
            oa << cx;
            oa << cy;
            oa << mbf;
            oa << mThDep;

            cout << " ...done" << std::endl;
            out.close();
        }

//        cout << "Printing all keyframes" << endl;
//
//        std::vector <KeyFrame *> allKfs = mpMap->GetAllKeyFrames();
//        for (KeyFrame * kf : allKfs)
//        {
//            cout << kf->mnId << " = " << kf->GetFirstConnection() << endl;
//        }
*/

    }


    void System::SaveMap(const string &filename) {
        /*
        cout << "Saving " << mpMap->MapPointsInMap() << " map points to PCD File...." << endl;
        std::vector <MapPoint *> ptrToMapPoints = mpMap->GetAllMapPoints();

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr (new pcl::PointCloud<pcl::PointXYZ>);

        cloudPtr->width = ptrToMapPoints.size();
        cloudPtr->height = 1;//1 for unorganized point cloud

        cloudPtr->points.resize(cloudPtr->width * cloudPtr->height);

        for(int counter = 0; counter < ptrToMapPoints.size(); counter++)
        {
            MapPoint* currentMapPoint = ptrToMapPoints.at(counter);

            cv::Mat worldPos = currentMapPoint->GetWorldPos();
            cloudPtr->points[counter].x = worldPos.at<float>(0,0);
            cloudPtr->points[counter].y = worldPos.at<float>(0,1);
            cloudPtr->points[counter].z = worldPos.at<float>(0,2);

        }


        string pcdFileName = filename;
        pcdFileName.replace(filename.find("."), 4,".pcd");

        pcl::io::savePCDFileASCII(pcdFileName, *cloudPtr);
        cout << "PCD file saved" << endl;


         */
//        std::ofstream out(filename + ".bin", std::ios_base::binary);


        std::ofstream out(filename);
        if (!out) {
            cerr << "Cannot Write to Mapfile: " << mapfile << std::endl;
            exit(-1);
        }


        cout << "Saving Mapfile: " << mapfile << std::flush;
        if (has_suffix(filename, ".txt") == true) {
            cout << "Saving mapfile in txt format" << endl;


            //Get next IDs for keyframe and map points
            KeyFrame *kf = mpMap->GetAllKeyFrames().at(0);
            nextMpID = GetAllMapPoints().at(0)->nNextId;
            nextKpID = kf->nNextId;

            //Get camera intrinsic properties of keyframes
            fx = kf->fx;
            fy = kf->fy;
            cx = kf->cx;
            cy = kf->cy;
            mbf = kf->mbf;
            mThDep = kf->mThDepth;

            //Copy map point observation indices to map point observation structure
            CopyMapPointIndicies ();


            cout << "Saving map file in bin format" << endl;
            cout << "Number of keyframes = " << GetNumberOfKeyframes() << endl;
            cout << "Number of map points = " << GetNumberOfMapPoints() << endl;
            cout << "Number of descriptors = " << GetNumberOfDescriptors() << endl;
            boost::archive::text_oarchive oa(out, boost::archive::no_header);




            string textHeader = "ORB SLAM Map File\n";
            oa << textHeader;


            //Serialize
            string mapFile = "\nmapFile\n";
            oa << mapFile;
            oa << mpMap;
            string smallVariables = "\nsmallVariables\n";
            oa << smallVariables;
            oa << nextMpID;
            oa << nextKpID;
            oa << fx;
            oa << fy;
            oa << cx;
            oa << cy;
            oa << mbf;
            oa << mThDep;

            cout << " ...done" << std::endl;
            out.close();


        } else {



            //Get next IDs for keyframe and map points
            KeyFrame *kf = mpMap->GetAllKeyFrames().at(0);
            nextMpID = GetAllMapPoints().at(0)->nNextId;
            nextKpID = kf->nNextId;

            cout << "Saving map file in bin format" << endl;
            cout << "Number of keyframes = " << GetNumberOfKeyframes() << endl;
            cout << "Number of map points = " << GetNumberOfMapPoints() << endl;
            cout << "Number of descriptors = " << GetNumberOfDescriptors() << endl;

            //Get camera intrinsic properties of keyframes
            fx = kf->fx;
            fy = kf->fy;
            cx = kf->cx;
            cy = kf->cy;
            mbf = kf->mbf;
            mThDep = kf->mThDepth;

            //Copy map point observation indicies to map point observation structure
            CopyMapPointIndicies ();


//            cout << "Saving map file in bin format" << endl;
//            cout << "Number of keyframes = " << GetNumberOfKeyframes() << endl;
//            cout << "Number of map points = " << GetNumberOfMapPoints() << endl;
//            cout << "Number of descriptors = " << GetNumberOfDescriptors() << endl;
            boost::archive::binary_oarchive oa(out, boost::archive::no_header);




            //Serialize

            oa << mpMap;






            //TODO: Uncomment these
            oa << nextMpID;
            oa << nextKpID;
            oa << fx;
            oa << fy;
            oa << cx;
            oa << cy;
            oa << mbf;
            oa << mThDep;

            cout << " ...done" << std::endl;
            out.close();
        }


        cout << "Saved map file to " << filename << endl;



    }


    void System::CopyMapPointIndicies ()
    {
        for (MapPoint *mp : GetAllMapPoints())
        {
            for (std::pair <KeyFrame*, size_t> obs : mp->GetObservations())
                mp->UpdateMpObservationIndex(obs.first, obs.second);
//                mp->GetMpObservations().find(obs.first)->second.index = obs.second;
        }

        cout << "Copied all map point indicies to MapPoint_Observation" << endl;
    }


    void System::ValidateMapPointObservationReconstruction ()
    {
        for (MapPoint *mp : GetAllMapPoints())
        {
            for (std::pair<KeyFrame * , MapPoint_Observation> mpObs : mp->GetMpObservations())
            {
                size_t GT = mp->GetObservations().find(mpObs.first)->second;
                if (mpObs.second.index != GT)
                {
                    cerr << "GT = " << GT << " our obs = " << mpObs.second.index << endl;
                }
//                    cerr << "We have a PROBLEM" << endl;
//                else
//                    cout << "all good" << endl;
            }
        }
        cout << "Validated all map point observations" << endl;
    }

    void System::ReconstructMapPointObservations (MapPoint *mp)
    {
        for (std::pair<KeyFrame*, MapPoint_Observation> mpObs : mp->GetMpObservations())
            mp->AppendObservations(mpObs.first, mpObs.second.index);

        //Estimating 2D coordinates in different keyframes

//        for (std::pair<KeyFrame*, MapPoint_Observation> mpObs : mp->GetMpObservations())
//        {
//
//        }

            return;
    }



    bool System::LoadMap(const string &filename, const string &cameraSettings, bool reconstruct) {

        std::ifstream in;
        bool isBinary;
        std::chrono::steady_clock::time_point begin;

        //Determine if the map file was saved in text format or binary format
        if (has_suffix(filename, ".txt")) {
            cout << "Loading txt map file" << endl;
            isBinary = false;
            begin = std::chrono::steady_clock::now();
            in.open(filename, std::ios_base::in);
        } else {
//            cout << "Loading binary map file" << endl;
            isBinary = true;
            begin = std::chrono::steady_clock::now();
            in.open(filename, std::ios_base::binary);
        }

        if (!in) {
            cerr << "Cannot Open Mapfile: " << filename << " , Create a new one" << std::endl;
            return false;
        }

        if (isBinary) {
            begin = std::chrono::steady_clock::now();
            boost::archive::binary_iarchive ia(in, boost::archive::no_header);
            ia >> mpMap;
            //TODO: Uncomment these
            ia >> nextMpID;
            ia >> nextKpID;
            ia >> fx;
            ia >> fy;
            ia >> cx;
            ia >> cy;
            ia >> mbf;
            ia >> mThDep;
        } else {
            begin = std::chrono::steady_clock::now();
            boost::archive::text_iarchive ia(in, boost::archive::no_header);
            string textHeader;
            ia >> textHeader;
            cout << textHeader;
            ia >> mpMap;

        }

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        //Reconstruct the map's data structures
        in.close();
        std::cout << "Map load time = "
                 << (std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()) / 1000000.0 << ", "
                  << " seconds" << std::endl;

        if (reconstruct == true)
        {
            cout << "Reconstructing the map" << endl;
            MapReconstruction(cameraSettings);

            unsigned int labelCounter [SEMANTIC_SEGMENTOR_NUMBER_OF_OBJECTS] = {0};
            mpMap->PrintMapPointLabelStatistics(labelCounter);

            cout << "Now constructing the KD tree structure" << endl;
            mpMap->ConstructKDTree();
            MapPoint *searchSpot = GetAllMapPoints().at(0);

            cout << "Now reconstructing the KD tree keyframe structure" << endl;


            float coordinates [3] = {searchSpot->GetWorldPos().at<float>(0,0), searchSpot->GetWorldPos().at<float>(1,0), searchSpot->GetWorldPos().at<float>(2,0)};
            mpMap->GetKDTreeNeighbors(coordinates, 1.0);


//            int labelCounter [SEMANTIC_SEGMENTOR_NUMBER_OF_OBJECTS] = {0};
//            mpMap->PrintMapPointLabelStatistics(labelCounter);
        }
        else {
            MapReconstruction (cameraSettings, reconstruct);
        }

        return true;
    }


    void System::MapPointKeyFrameRelations(MapPoint *mp) {

//        std::vector < unsigned long int > kfIDs;
//
//        for (std::pair <KeyFrame*, MapPoint_Observation> mpObs : mp->mpObservation)
//            kfIDs.push_back(mpObs.first->mnId);
//
//        if (kfIDs != mp->keyframeIDs)
//            cerr << "Map point = " << mp->mnId << endl;// ", keyframes = ";
//        else
//            cout << "Matched for mp = " << mp->mnId << endl;

    }



    void System::MapReconstruction(const string &strSettingPath, bool reconstruct) {

//        cout << "Map reconstruction" << endl;
        //Setup pointers to ORB vocabulary and the keyframe database
        //Reconstruct the keyframe database as well
        const int CC_ARRAY_SIZE = 6;
        float CC_Array[CC_ARRAY_SIZE] = {fx, fy, cx, cy, mbf, mThDep};
//        cout << "MBF = " << mbf << endl;

        std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
        mpKeyFrameDatabase = new KeyFrameDatabase(mpVocabulary);
//        cout << "Here?" << endl;
        vector<ORB_SLAM2::KeyFrame *> allKeyFrames = mpMap->GetAllKeyFrames();
//        cout << "Or, here?" << endl;
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        int nFeatures = fSettings["ORBextractor.nFeatures"];
        float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
        int nLevels = fSettings["ORBextractor.nLevels"];
        int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
        int fMinThFAST = fSettings["ORBextractor.minThFAST"];
        ORBextractor *OrbExtractor = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
        // Scale Level Info
        int mnScaleLevels = OrbExtractor->GetLevels();
        float mfScaleFactor = OrbExtractor->GetScaleFactor();
        float mfLogScaleFactor = log(mfScaleFactor);
        std::vector <float> mvScaleFactors = OrbExtractor->GetScaleFactors();
        std::vector <float> mvInvScaleFactors = OrbExtractor->GetInverseScaleFactors();
        std::vector <float> mvLevelSigma2 = OrbExtractor->GetScaleSigmaSquares();
        std::vector <float> mvInvLevelSigma2 = OrbExtractor->GetInverseScaleSigmaSquares();
        std::map <unsigned long int, KeyFrame * > orderedKeyFrames;
        //////////////////////////////// RECONSTRUCTION ///////////////////////////////////////////////
        //Reconstruct keyframe information
        unsigned int mps = 0;


        std::chrono::steady_clock::time_point keyframeStart, keyframeEnd, mpStart, mpEnd, cgStart, cgEnd;



        keyframeStart = std::chrono::steady_clock::now();


//        cout << "Map reconstruction before else statement" << endl;
        if (reconstruct == true) {

            cout << "Keyframe reconstruction ...." << endl;
            for (KeyFrame *keyframe : allKeyFrames) {
                // Keyframe database
                keyframe->SetORBvocabulary(mpVocabulary);//set pointer to ORB vocabulary
                keyframe->SetKeyFrameDB(mpKeyFrameDatabase);//set pointer to keyframe database
                // Bag of words
                std::chrono::steady_clock::time_point bowStart = std::chrono::steady_clock::now();
                keyframe->ReconstructBoW();//reconstruct BoW from map points
                std::chrono::steady_clock::time_point bowEnd = std::chrono::steady_clock::now();
                //Keyframe database
                mpKeyFrameDatabase->add(keyframe);//add keyframe to keyframe database
                std::chrono::steady_clock::time_point dbEnd = std::chrono::steady_clock::now();
                // Redundant data
                keyframe->SetMapPointer(mpMap);//set pointer to mpMap
                keyframe->SetCameraCalibration(CC_Array);//restore camera calibration for the keyframe
                keyframe->nNextId = nextKpID;//set ID for next keyframe
                keyframe->AssignGridParameters();
                keyframe->AssignScaleVariables(mnScaleLevels, mfScaleFactor, mfLogScaleFactor, mvScaleFactors,
                                               mvInvScaleFactors, mvLevelSigma2, mvInvLevelSigma2);
                std::chrono::steady_clock::time_point assignEnd = std::chrono::steady_clock::now();
                //Map point observations
                keyframe->StoreMapPointObservation();
                //Initialize outliers
                keyframe->InitOutliers();
                if (keyframe != NULL)
                    if (keyframe->isBadReconstruction() == false)
                        orderedKeyFrames.insert(std::pair<unsigned long int, KeyFrame *>(keyframe->mnId, keyframe));

            }


            keyframeEnd = std::chrono::steady_clock::now();

            //Reconstruct map point information

            const int AXES = 4;
            unsigned long int numberOfObservations = 0;
            unsigned long int numberOfMpObs = 0;
            unsigned int normalDuration = 0;
//            cout << "Map point reconstruction ...." << endl;


            mpStart = std::chrono::steady_clock::now();

            for (MapPoint *mp : mpMap->GetAllMapPoints()) {
                mp->SetMapPointer(mpMap);//set pointer to mpMap
                mp->nNextId = nextMpID;//set ID for next map point.
                mp->ReconstructMpObservation();
                ReconstructMapPointObservations(mp);//copy map point indicies from mpObservation to observation
                mp->UpdateNormalAndDepth();//reconstruct normal vector

                numberOfMpObs += mp->GetMpObservations().size();
                numberOfObservations += mp->GetObservations().size();
            }
            mpEnd = std::chrono::steady_clock::now();

            cgStart = std::chrono::steady_clock::now();



            for (unsigned long int counter = 0; counter < orderedKeyFrames.size(); counter++) {
                KeyFrame *kf = orderedKeyFrames.find(counter)->second;
//                cout << counter << endl;

//                if (counter == 1 || counter == 4 || counter == 47 || counter == 50)
//                    continue;
//                if (counter >= 190 && counter <= 200) {
//                    std::vector <KeyFrame*>::iterator it = orderedKeyFrames.be
//                    orderedKeyFrames.erase (orderedKeyFrames.begin()->first + counter);
//                    continue;
//                }
//                if (counter == 6 || counter == 31)
//                    continue;
                if (kf == NULL)
                    continue;
                if (kf->isBadReconstruction())
                    continue;
                if (kf->mnId > mpMap->KeyFramesInMap()) {
//                    cout << "Skipping " << kf->mnId << endl;
                    continue;
                } else
                    kf->ReconstructCoVisibilityGraph();
            }

            cgEnd = std::chrono::steady_clock::now();

        }


        else
        {
            cout << "Partial reconstruction" << endl;

            const int CC_ARRAY_SIZE = 6;
            float CC_Array[CC_ARRAY_SIZE] = {fx, fy, cx, cy, mbf, mThDep};

            std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
            mpKeyFrameDatabase = new KeyFrameDatabase(mpVocabulary);
            vector<ORB_SLAM2::KeyFrame *> allKeyFrames = mpMap->GetAllKeyFrames();
            cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
            int nFeatures = fSettings["ORBextractor.nFeatures"];
            float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
            int nLevels = fSettings["ORBextractor.nLevels"];
            int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
            int fMinThFAST = fSettings["ORBextractor.minThFAST"];
            ORBextractor *OrbExtractor = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
            // Scale Level Info
            int mnScaleLevels = OrbExtractor->GetLevels();
            float mfScaleFactor = OrbExtractor->GetScaleFactor();
            float mfLogScaleFactor = log(mfScaleFactor);
            std::vector <float> mvScaleFactors = OrbExtractor->GetScaleFactors();
            std::vector <float> mvInvScaleFactors = OrbExtractor->GetInverseScaleFactors();
            std::vector <float> mvLevelSigma2 = OrbExtractor->GetScaleSigmaSquares();
            std::vector <float> mvInvLevelSigma2 = OrbExtractor->GetInverseScaleSigmaSquares();


            cout << "Keyframe reconstruction" << endl;
            keyframeStart = std::chrono::steady_clock::now();

            for (KeyFrame *keyframe : allKeyFrames) {
                // Keyframe database
                keyframe->SetORBvocabulary(mpVocabulary);
                // Bag of words
                keyframe->ReconstructBoW();
                // Camera calibration
                keyframe->SetCameraCalibration(CC_Array);
                // Assign grid values and poses
                keyframe->AssignGridParameters();
                // Assign scale variables
                keyframe->AssignScaleVariables(mnScaleLevels, mfScaleFactor, mfLogScaleFactor, mvScaleFactors,
                                               mvInvScaleFactors, mvLevelSigma2, mvInvLevelSigma2);
                // Initialize outliers
                keyframe->InitOutliers();
                // Record map point observations
                keyframe->StoreMapPointObservation();
            }
            keyframeEnd = std::chrono::steady_clock::now();

//            cout << "Done with keyframe reconstruction" << endl;

//            cout << "Reconstructing map point observations" << endl;
            mpStart = std::chrono::steady_clock::now();
            for (MapPoint * mp : mpMap->GetAllMapPoints ())
            {
                mp->ReconstructMpObservation ();
                ReconstructMapPointObservations (mp);

            }
            mpEnd = std::chrono::steady_clock::now();

//            cout << "Done reconstructing map points" << endl;

        }

//        PrintKeyFramesAndMapPoints();






//        cout << "Number of connections per keyframe = " << numberOfConnections / (mpMap->GetAllKeyFrames().size() * 1.0) << endl;
//        cout << "Average weight per keyframes = " << averageWeights / (mpMap->GetAllKeyFrames().size() * 1.0) << endl;
//        cout << "KF duration = " << kfDuration / 1000000.0 << endl;
//        cout << "Pair forming duration = " << pairsDuration / 1000000.0 << endl;
//        cout << "Sorting duration = " << sortingDuration / 1000000.0 << endl;
//        cout << "Update duration = " << updateDuration / 1000000.0 << endl;
//        cout << "Update best co-visibles duration = " << newUpdateDuration / 1000000.0 << endl;
//        std::chrono::steady_clock::time_point errorStart = std::chrono::steady_clock::now();
//        cout << "Total difference = " << difference / mpMap->GetAllMapPoints().size() << endl;
//        cout << "Reconstructed" << endl;
//        std::chrono::steady_clock::time_point errorEnd = std::chrono::steady_clock::now();
//        std::cout << "Error compenstation = "
//                  << (std::chrono::duration_cast<std::chrono::microseconds>(errorEnd - errorStart).count()) / 1000000.0
//                  << " seconds" << std::endl;
//        cout << "Reconstruction complete" << endl;
//        cout << "******* Reconstruction Statistics ************" << endl;
//        cout << "Number of keyframes = " << GetNumberOfKeyframes() << endl;
//        cout << "Number of map points = " << GetNumberOfMapPoints() << endl;
//        cout << "Number of descriptors = " << GetNumberOfDescriptors() << endl;
//        cout << endl;
//        cout << "Total difference = " << difference / mpMap->GetAllMapPoints().size() << endl;
//        cout << "Reconstructed" << endl;
//        cout << "Freed orbextractor" << endl;


        std::chrono::steady_clock::time_point orbSlamStart = std::chrono::steady_clock::now();
//        for (unsigned long int counter = 0; counter < orderedKeyFrames.size(); counter++)
//        {
//            KeyFrame *kf = orderedKeyFrames.find(counter)->second;
//
//            if (kf == NULL)
//                continue;
//            if (kf->isBad())
//                continue;
//            if (kf->mnId > mpMap->KeyFramesInMap())
//            {
//                cout << "Skipping " << kf->mnId << endl;
//                continue;
//            }
//            else
//                kf->OldReconstructCoVisibilityGraph();
//        }
        std::chrono::steady_clock::time_point orbSlamEnd = std::chrono::steady_clock::now();
        /////////////////////////////////////////////////////////////////////////////////////////////
        unsigned int kfDuration = std::chrono::duration_cast <std::chrono::microseconds> (keyframeEnd - keyframeStart).count();
        unsigned int mpDuration = std::chrono::duration_cast <std::chrono::microseconds> (mpEnd - mpStart).count();
//        unsigned int orbSlamDuration = std::chrono::duration_cast <std::chrono::microseconds> (orbSlamEnd - orbSlamStart).count();
        unsigned int cgDuration = std::chrono::duration_cast <std::chrono::microseconds> (cgEnd - cgStart).count();
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "Map reconstruction time = ";
        std::cout << (std::chrono::duration_cast<std::chrono::microseconds>( orbSlamStart - keyframeStart).count()) / 1000000.0 << endl ;
//        cout << bowDuration / 1000000.0 << ", " << dbDuration / 1000000.0 << ", " << assignDuration / 1000000.0 << ", " << normalDuration / 1000000.0 << ", " << obsDuration / 1000000.0 <<        ", " << cgDuration / 1000000.0 << ", " << kfDuration / 1000000.0 << ", " << mpDuration / 1000000.0 << ", ,";
        cout << "Keyframe = " << kfDuration / 1000000.0 << endl;
        cout << "CG duration = " << cgDuration / 1000000.0 << endl;
//        cout << "Old CG duration = " << (std::chrono::duration_cast<std::chrono::microseconds>( orbSlamEnd - orbSlamStart).count()) / 1000000.0 << endl;

//        cout << "DB = " << dbDuration / 1000000.0 << endl;
//        cout << "Assign = " << assignDuration / 1000000.0 << endl;
        cout << "Map point = " << mpDuration / 1000000.0 << endl;

//        cout << "Normal = " << normalDuration / 1000000.0 << endl;
//        cout << "Observation reconstruction = " << obsDuration / 1000000.0 << endl;
//        cout << "ORB-SLAM2 reconstruction = " << orbSlamDuration / 1000000.0 << endl;
//        cout << "Austerity reconstruction = " << austerityDuration / 1000000.0 << endl;
//        float numberOfConnections = 0;
//        float averageWeights = 0;
//        unsigned long int kfDuration = 0;
//        unsigned long int pairsDuration = 0;
//        unsigned long int sortingDuration = 0;
//        unsigned long int updateDuration = 0;
//        unsigned long int newUpdateDuration = 0;
//        for (KeyFrame *kf : mpMap->GetAllKeyFrames())
//        {
//            numberOfConnections += kf->GetConnectedKeyFrames().size();
//            averageWeights += kf->GetAverageConnectedKeyFrameWeight();
//            kfDuration += kf->kfDuration;
//            pairsDuration += kf->pairDuration;
//            sortingDuration += kf->sortingDuration;
//            updateDuration += kf->updateDuration;
//            newUpdateDuration += kf->updateBestCovisibles;
//        }
//        cout << "Number of connections per keyframe = " << numberOfConnections / (mpMap->GetAllKeyFrames().size() * 1.0) << endl;
//        cout << "Average weight per keyframes = " << averageWeights / (mpMap->GetAllKeyFrames().size() * 1.0) << endl;
//        cout << "KF duration = " << kfDuration / 1000000.0 << endl;
//        cout << "Pair forming duration = " << pairsDuration / 1000000.0 << endl;
//        cout << "Sorting duration = " << sortingDuration / 1000000.0 << endl;
//        cout << "Update duration = " << updateDuration / 1000000.0 << endl;
//        cout << "Update best co-visibles duration = " << newUpdateDuration / 1000000.0 << endl;
//        std::chrono::steady_clock::time_point errorStart = std::chrono::steady_clock::now();
//        cout << "Total difference = " << difference / mpMap->GetAllMapPoints().size() << endl;
//        cout << "Reconstructed" << endl;
//        std::chrono::steady_clock::time_point errorEnd = std::chrono::steady_clock::now();
//        std::cout << "Error compenstation = "
//                  << (std::chrono::duration_cast<std::chrono::microseconds>(errorEnd - errorStart).count()) / 1000000.0
//                  << " seconds" << std::endl;
//        cout << "Reconstruction complete" << endl;
//        cout << "******* Reconstruction Statistics ************" << endl;
//        cout << "Number of keyframes = " << GetNumberOfKeyframes() << endl;
//        cout << "Number of map points = " << GetNumberOfMapPoints() << endl;
//        cout << "Number of descriptors = " << GetNumberOfDescriptors() << endl;
//        cout << endl;
//        cout << "Total difference = " << difference / mpMap->GetAllMapPoints().size() << endl;
//        cout << "Reconstructed" << endl;
//        cout << "Freed orbextractor" << endl;


        cout << "******* Map statistics ***********" << endl;
        cout << "Number of keyframes = " << GetAllKeyFrames().size() << endl;
        cout << "Number of map-points = " << GetAllMapPoints().size() << endl;
        cout << "**********************************" << endl;
        DiffOperationMapPointOriginsTestCase();



        delete (OrbExtractor);
        return;

    }

    void System::DiffOperationMapPointOriginsTestCase ()
    {
        cout << "**************************************" << endl;
        cout << "Testing map origins for diff operation" << endl;
        cout << "Checking for mis labeled map-points" << endl;
        vector <MapPoint*> allMps = mpMap->GetAllMapPoints();
        for ( MapPoint * mp : allMps ) {
            if (mp->GetFromPreviousSession() == false) {
                cerr << "Made a mistake here" << endl;
            }

            if (mp->GetNumberOfOccurencesInNewSession() != 0)
            {
                cerr << "Made another mistake here" << endl;
            }
        }

        cout << "************* Passed *****************" << endl;
        cout << "**************************************" << endl;


        return;
    }


    unsigned long System::GetNumberOfKeyframes() {
        return mpMap->KeyFramesInMap();
    }

    unsigned long System::GetNumberOfMapPoints() {
        return mpMap->GetAllMapPoints().size();
    }


    unsigned long System::GetNumberOfDescriptors() {
        unsigned long descriptorCounter = 0;
        for (ORB_SLAM2::MapPoint *mp:mpMap->GetAllMapPoints()) {
            for (std::pair<ORB_SLAM2::KeyFrame *, MapPoint_Observation> descriptor:mp->mpObservation)
                descriptorCounter++;
        }

        return descriptorCounter;
    }

    void System::CompareObservationsAndDescriptorList() {
        /*
        for (ORB_SLAM2::MapPoint* mp:mpMap->GetAllMapPoints())
        {
            cout << mp->mnId << ", descriptorList = " << mp->mpObservation.size() << ", observations = " << mp->nObs << endl;
        }
         */
    }


    void System::PrintKeyFramesAndMapPoints(void) {

        static int counter = 0;
        static ofstream outputFile;

        if (counter == 0)
            outputFile.open("KeyFramesDump.txt");

        outputFile << "*********************************************" << endl <<
                   "********************************************" << endl << "*************"
                           "Frame " << counter++ << endl << endl << endl;
        const unsigned long int numberOfKeyFrames = mpMap->GetAllKeyFrames().size();
        const unsigned long int numberOfMapPoints = mpMap->GetAllMapPoints().size();

        outputFile << "Number of keyframes = " << numberOfKeyFrames << ", number of map points = " << numberOfMapPoints
                   << endl;

        for (int kfCounter = 0; kfCounter < numberOfKeyFrames; kfCounter++) {
            ORB_SLAM2::KeyFrame *currentKF = mpMap->GetAllKeyFrames().at(kfCounter);

            outputFile << "KF : " << kfCounter << ", with " << currentKF->GetMapPoints().size() << " map points"
                       << endl;
            const int numberOfMPinCurrentKF = currentKF->GetMapPointMatches().size();

            for (int mpCounter = 0; mpCounter < numberOfMPinCurrentKF; mpCounter++) {
                ORB_SLAM2::MapPoint *currentMP = currentKF->GetMapPointMatches().at(mpCounter);

                //Do we have a map point in this location? If null, then skip
                if (currentMP != NULL) {

                    cv::KeyPoint KP, unKP;
                    int numberOfObservations = currentMP->nObs;//Find the number of observations

                    //If greater than one, then get the observation for the current keyframe
//                    if (numberOfObservations > 1)
//                    {
                    KP = currentMP->mpObservation.find(currentKF)->second.Keypoint;
                    unKP = currentMP->mpObservation.find(currentKF)->second.Keypoint;



                    //Find the x and y differences
                    float xDiff = abs(currentKF->mvKeysUn[mpCounter].pt.x - unKP.pt.x);
                    float yDiff = abs(currentKF->mvKeysUn[mpCounter].pt.y - unKP.pt.y);

                    float x, y;
                    x = y = 0;



                    //If there is a difference, between the values
                    if (xDiff || yDiff) {
                        //If the number of observations is greater than one
                        std::map<KeyFrame *, size_t> observations = currentMP->GetObservations();

                        outputFile << "Difference!!!!! MP : " << mpCounter <<
                                   ", x : " << currentKF->mvKeysUn[mpCounter].pt.x << " - " << unKP.pt.x
                                   << " ------ y: " << currentKF->mvKeysUn[mpCounter].pt.y << " - " << unKP.pt.y
                                   << endl;

                        /*

                            int counter = 0;
                            for (std::map<KeyFrame*, size_t>::iterator it = observations.begin(); it != observations.end(); it++)
                            {
                                xDiff = abs (currentMP->unKP.pt.x - it->first->mvKeysUn[it->second].pt.x);
                                yDiff = abs (currentMP->unKP.pt.y - it->first->mvKeysUn[it->second].pt.y);

                                counter++;
                                if (xDiff == 0 && yDiff == 0)
                                {
                                    x = it->first->mvKeysUn[it->second].pt.x;
                                    y = it->first->mvKeysUn[it->second].pt.y;
                                    outputFile << "Found a match" << endl;
                                    break;
                                }
                            }

                            outputFile << "No diff MP : " << mpCounter << ", x : " << currentMP->unKP.pt.x
                                       << " - " << x << " ------ y: "
                                       << currentMP->unKP.pt.y << " - " << y
                                       << endl;

                        */
                    } else {
                        outputFile << "MP : " << mpCounter <<
                                   ", x : " << currentKF->mvKeysUn[mpCounter].pt.x << " - " << unKP.pt.x
                                   << " ------ y: " << currentKF->mvKeysUn[mpCounter].pt.y << " - " << unKP.pt.y
                                   << endl;
                    }
                } else
                    outputFile << "MP : " << mpCounter << endl;
            }
//                    outputFile << "No Map Point at  : " << mpCounter << ", x :" << currentKF->mvKeysUn[mpCounter].pt.x
//                               << " - null" << " ------ y: " << currentKF->mvKeysUn[mpCounter].pt.y << " - null" << endl;
        }

        outputFile << " ---------------------------- " << endl << endl;
    }

    void System::PrintKeyFrameExistencePoints() {
        cout << "********* Printing keyframe origination points **************" << endl;
        std::vector<ORB_SLAM2::KeyFrame *> allKFs = mpMap->GetAllKeyFrames();
        int loadedFromDisk = 0;
        int createdNow = 0;

        int kfCounter = 0;
        for (ORB_SLAM2::KeyFrame *kf:allKFs) {
            kfCounter++;
            if (kf->isLoadedFromDisk) {
                cout << kf->mnId << " : loaded from disk" << endl;
                loadedFromDisk++;
            } else {
                createdNow++;
                cout << kf->mnId << " : created now" << endl;
            }
        }


        cout << "**************  Statistics  ******************" << endl;
        cout << "Loaded from disk = " << loadedFromDisk << endl;
        cout << "Created now = " << createdNow << endl;
        int descriptorCounter = 0;
        int numberOfMapPoints = 0;

        for (ORB_SLAM2::MapPoint *mp:mpMap->GetAllMapPoints()) {
            numberOfMapPoints++;
            for (std::pair<ORB_SLAM2::KeyFrame *, MapPoint_Observation> descriptor:mp->mpObservation)
                descriptorCounter++;
        }

        cout << "Number of map points = " << numberOfMapPoints << endl;
        cout << "Number of descriptors saved = " << descriptorCounter << endl;
    }


    std::vector<ORB_SLAM2::MapPoint *> System::GetAllMapPoints(void) {
        return mpMap->GetAllMapPoints();
    }

    void System::PrintDescriptorDistances(void) {
        std::vector<ORB_SLAM2::MapPoint *> allMapPoints = GetAllMapPoints();
        int below50 = 0;
        int above50 = 0;
        float max = 0;
        float averageDistance = 0;
        int counter = 0;
        for (MapPoint *mapPoint:allMapPoints)//iterate through all the map points
        {
            std::map<KeyFrame *, MapPoint_Observation> mpObs = mapPoint->mpObservation;

            cv::Mat referenceDescriptor;
            float descriptorDistance = 0;


            for (std::pair<KeyFrame *, MapPoint_Observation> obs:mpObs)//iterate through all the observations of the map points
            {
                if (obs.first == mpObs.begin()->first) {
                    referenceDescriptor = mapPoint->GetDescriptor();
                }
                descriptorDistance += GetDescriptorDistance(obs.second.descriptor, referenceDescriptor);
            }

            float distance = (descriptorDistance * 1.0) / mapPoint->mpObservation.size();

            if (distance <= 50)
                below50++;
            else {
                above50++;
            }

            if (distance > max)
                max = distance;

            cout << "mp = " << mapPoint->mnId << ", obs =  "
                 << mapPoint->mpObservation.size() << ", distance  = "
                 << (descriptorDistance * 1.0) / mapPoint->mpObservation.size() << endl;


            averageDistance += distance;
            counter++;


        }

        cout << "Below threshold = " << below50 << endl;
        cout << "Above threshold = " << above50 << endl;

        cout << "Max = " << max << endl;
        cout << "Average = " << averageDistance / counter << endl;


    }

    int System::GetDescriptorDistance(cv::Mat a, cv::Mat b) {
        ORB_SLAM2::ORBmatcher matcher;
        return matcher.DescriptorDistance(a, b);
    }


} //namespace ORB_SLAM
