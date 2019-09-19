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


#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"
#include "SemanticSegmentor.h"


#include<iostream>

#include<mutex>
#include<chrono>


using namespace std;

namespace ORB_SLAM2 {



    Tracking::Tracking(System *pSys, ORBVocabulary *pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap,
                       KeyFrameDatabase *pKFDB, const string &strSettingPath, const int sensor,  bool isDynamicObjectRemoval, bool majorityVoting, bool isRobustFeatureSearch, bool bReuseMap) :
            mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
            mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer *>(NULL)), mpSystem(pSys), mpViewer(NULL),
            mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0), tIsDynamicObjectRemoval (isDynamicObjectRemoval), RobustFeatureSearch (isRobustFeatureSearch),
            MajorityVoting (majorityVoting)
    {
        // Load camera parameters from settings file
        SetThreadRunning();
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        float fx = fSettings["Camera.fx"];
        float fy = fSettings["Camera.fy"];
        float cx = fSettings["Camera.cx"];
        float cy = fSettings["Camera.cy"];

        cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
        K.at<float>(0, 0) = fx;
        K.at<float>(1, 1) = fy;
        K.at<float>(0, 2) = cx;
        K.at<float>(1, 2) = cy;
        K.copyTo(mK);

        cv::Mat DistCoef(4, 1, CV_32F);
        DistCoef.at<float>(0) = fSettings["Camera.k1"];
        DistCoef.at<float>(1) = fSettings["Camera.k2"];
        DistCoef.at<float>(2) = fSettings["Camera.p1"];
        DistCoef.at<float>(3) = fSettings["Camera.p2"];
        const float k3 = fSettings["Camera.k3"];
        if (k3 != 0) {
            DistCoef.resize(5);
            DistCoef.at<float>(4) = k3;
        }
        DistCoef.copyTo(mDistCoef);

        mbf = fSettings["Camera.bf"];

        float fps = fSettings["Camera.fps"];
        if (fps == 0)
            fps = 30;

        // Max/Min Frames to insert keyframes and to check relocalisation
        mMinFrames = 0;
        mMaxFrames = fps;
//
//    if (!EVALUATION_MODE) {
//        cout << endl << "Camera Parameters: " << endl;
//        cout << "- fx: " << fx << endl;
//        cout << "- fy: " << fy << endl;
//        cout << "- cx: " << cx << endl;
//        cout << "- cy: " << cy << endl;
//        cout << "- k1: " << DistCoef.at<float>(0) << endl;
//        cout << "- k2: " << DistCoef.at<float>(1) << endl;
//        if (DistCoef.rows == 5)
//            cout << "- k3: " << DistCoef.at<float>(4) << endl;
//        cout << "- p1: " << DistCoef.at<float>(2) << endl;
//        cout << "- p2: " << DistCoef.at<float>(3) << endl;
//        cout << "- fps: " << fps << endl;
//
//    }
        int nRGB = fSettings["Camera.RGB"];
        mbRGB = nRGB;
//    if (!EVALUATION_MODE) {
//        if (mbRGB)
//            cout << "- color order: RGB (ignored if grayscale)" << endl;
//        else
//            cout << "- color order: BGR (ignored if grayscale)" << endl;
//    }

        // Load ORB parameters

        int nFeatures = fSettings["ORBextractor.nFeatures"];
        float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
        int nLevels = fSettings["ORBextractor.nLevels"];
        int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
        int fMinThFAST = fSettings["ORBextractor.minThFAST"];

        mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        if (sensor == System::STEREO)
            mpORBextractorRight = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        if (sensor == System::MONOCULAR)
            mpIniORBextractor = new ORBextractor(2 * nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

//    if (!EVALUATION_MODE) {

//        cout << endl << "ORB Extractor Parameters: " << endl;
//        cout << "- Number of Features: " << nFeatures << endl;
//        cout << "- Scale Levels: " << nLevels << endl;
//        cout << "- Scale Factor: " << fScaleFactor << endl;
//        cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
//        cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;
//    }
        if (sensor == System::STEREO || sensor == System::RGBD) {
            mThDepth = mbf * (float) fSettings["ThDepth"] / fx;
            cout << "Threshold depth = " << mThDepth << endl;
//        if (!EVALUATION_MODE)


//        cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;

        }


        KpLog.open ("Datasets/KpLog_1.txt");

        if (sensor == System::RGBD) {
            mDepthMapFactor = fSettings["DepthMapFactor"];
            if (fabs(mDepthMapFactor) < 1e-5)
                mDepthMapFactor = 1;
            else
                mDepthMapFactor = 1.0f / mDepthMapFactor;
        }
        if (bReuseMap) {
            mState = LOST;
            PerformDiffOperation = true;
        }
        else
            PerformDiffOperation = false;

//        cout << "Done with tracking" << endl;
    }

    void Tracking::SetLocalMapper(LocalMapping *pLocalMapper) {
        mpLocalMapper = pLocalMapper;
    }

    void Tracking::SetLoopClosing(LoopClosing *pLoopClosing) {
        mpLoopClosing = pLoopClosing;
    }

    void Tracking::SetViewer(Viewer *pViewer) {
        mpViewer = pViewer;
    }

    void Tracking::SetThreadRunning() {
        threadRunning = true;
    }

    void Tracking::UnsetThreadRunning ()
    {
        threadRunning = false;
    }

    bool Tracking::GetThreadStatus() {
        return threadRunning;
    }


    cv::Mat Tracking::GrabStereoKeyFrame(ORB_SLAM2::KeyFrame *kf, bool isAnchorPointStitching) {

        cv::Mat poseOfKeyFrame = StitchWithMap(kf, isAnchorPointStitching);

        if (!poseOfKeyFrame.empty())
        {
            mpLocalMapper->CheckAndProcessNewKeyFrame();
            cout << "Processed new keyframe" << endl;
        }

        return poseOfKeyFrame;
    }


    cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp,
                                      ofstream *featureFile) {
        //Profiling


        static float time = 0;
        static int counter = 0;


        mImGray = imRectLeft;
        mImOverlay = imRectLeft;
        cv::Mat imGrayRight = imRectRight;

        if (mImGray.channels() == 3) {
            if (mbRGB) {
                cvtColor(mImGray, mImGray, CV_RGB2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_RGB2GRAY);
            } else {
                cvtColor(mImGray, mImGray, CV_BGR2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_BGR2GRAY);
            }
        } else if (mImGray.channels() == 4) {
            if (mbRGB) {
                cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_RGBA2GRAY);
            } else {
                cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_BGRA2GRAY);
            }
        }

//    if (FEATURE_DEBUG_MODE)
//        cout << "TrackStereo:GrabImageStereo:Frame" << endl;



        mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary,
                              mK, mDistCoef, mbf, mThDepth, featureFile);



        //Frame forms a data structure from mImGray and imGrayRight
        //1. First it extracts ORB features from the two eyes
        //2. The same features are extracted from the two and stored in mvKeys
        //3. The features/keypoints are undistorted
        //4. The keypoints are then passed through a stereo matching algorithm which computes their depths
        //5. Thus, mvKeys contain the keypoints in terms of their x, y and mvDepth contain their depth in z coordinates




        std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
        Track();
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        time += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
                counter++;

//        cout << time / ( counter * 1000000.0 ) << endl;



        //CODE FOR FINDING TRAJECTORY MISSING FRAMES
//    if (mCurrentFrame.mTcw.empty() == false)
//        return mCurrentFrame.mTcw * mCurrentFrame.mpReferenceKF->GetPoseInverse();
//    else
        return mCurrentFrame.mTcw.clone();

        //Uncomment this
//    return mCurrentFrame.mTcw.clone();
    }



    cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const cv::Mat &imLabel, const cv::Mat &imOverlay, const double &timestamp,
                                      ofstream *featureFile) {
        //Profiling

        static float time = 0;
        static int counter = 0;

        SetThreadRunning();

        mImGray = imRectLeft;
        mImOverlay = imRectLeft;
        mImLabel = imLabel;

        cv::Mat imGrayRight = imRectRight;

        if (mImGray.channels() == 3) {
            if (mbRGB) {
                cvtColor(mImGray, mImGray, CV_RGB2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_RGB2GRAY);
            } else {
                cvtColor(mImGray, mImGray, CV_BGR2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_BGR2GRAY);
            }
        } else if (mImGray.channels() == 4) {
            if (mbRGB) {
                cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_RGBA2GRAY);
            } else {
                cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_BGRA2GRAY);
            }
        }


        //Frame forms a data structure from mImGray and imGrayRight
        //1. First it extracts ORB features from the two eyes
        //2. The same features are extracted from the two and stored in mvKeys
        //3. The features/keypoints are undistorted
        //4. The keypoints are then passed through a stereo matching algorithm which computes their depths
        //5. Thus, mvKeys contain the keypoints in terms of their x, y and mvDepth contain their depth in z coordinates
        mCurrentFrame = Frame(mImGray, imGrayRight, imLabel, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary,
                              mK, mDistCoef, mbf, mThDepth, featureFile);

//        cout << "Number of features: " << mCurrentFrame.mDescriptors.size();



        Track();
        counter++;

        UnsetThreadRunning();
        return mCurrentFrame.mTcw.clone();

        //Uncomment this
//    return mCurrentFrame.mTcw.clone();
    }


    cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const cv::Mat &imLabel,
                                      const cv::Mat &imOverlay, const cv::Mat &imDP, const cv::Mat &imMN, const cv::Mat &imFDP,
                                      const double &timestamp, ofstream *featureFile) {
        //Profiling


        SetThreadRunning();

        mImGray = imRectLeft;
        mImOverlay = imRectLeft;
        mImLabel = imLabel;

        cv::Mat imGrayRight = imRectRight;

        if (mImGray.channels() == 3) {
            if (mbRGB) {
                cvtColor(mImGray, mImGray, CV_RGB2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_RGB2GRAY);
            } else {
                cvtColor(mImGray, mImGray, CV_BGR2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_BGR2GRAY);
            }
        } else if (mImGray.channels() == 4) {
            if (mbRGB) {
                cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_RGBA2GRAY);
            } else {
                cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_BGRA2GRAY);
            }
        }


        //Frame forms a data structure from mImGray and imGrayRight
        //1. First it extracts ORB features from the two eyes
        //2. The same features are extracted from the two and stored in mvKeys
        //3. The features/keypoints are undistorted
        //4. The keypoints are then passed through a stereo matching algorithm which computes their depths
        //5. Thus, mvKeys contain the keypoints in terms of their x, y and mvDepth contain their depth in z coordinates
        mCurrentFrame = Frame(mImGray, imGrayRight, imLabel, imDP, imMN, imFDP, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary,
                              mK, mDistCoef, mbf, mThDepth, featureFile);


//        cout << "Saving to disk" << endl;
//        mCurrentFrame.SaveFrameToDisk (1, &KpLog);
        Track();

        //CODE FOR FINDING TRAJECTORY MISSING FRAMES
//    if (mCurrentFrame.mTcw.empty() == false)
//        return mCurrentFrame.mTcw * mCurrentFrame.mpReferenceKF->GetPoseInverse();
//    else


        UnsetThreadRunning();
//        cout << "Tracking stopped running" << endl;
        return mCurrentFrame.mTcw.clone();

        //Uncomment this
//    return mCurrentFrame.mTcw.clone();
    }




    cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight,
                                      const cv::Mat &imLabel, const cv::Mat &imOverlay,
                                      const cv::Mat &imDepth, const double &timestamp,
                                      ofstream *featureFile) {
        //Profiling

        SetThreadRunning();
        mImGray = imRectLeft;
        mImOverlay = imRectLeft;
        mImLabel = imLabel;
        mImDepth = imDepth;

        cv::Mat imGrayRight = imRectRight;

        if (mImGray.channels() == 3) {
            if (mbRGB) {
                cvtColor(mImGray, mImGray, CV_RGB2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_RGB2GRAY);
            } else {
                cvtColor(mImGray, mImGray, CV_BGR2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_BGR2GRAY);
            }
        } else if (mImGray.channels() == 4) {
            if (mbRGB) {
                cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_RGBA2GRAY);
            } else {
                cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_BGRA2GRAY);
            }
        }
        //Frame forms a data structure from mImGray and imGrayRight
        //1. First it extracts ORB features from the two eyes
        //2. The same features are extracted from the two and stored in mvKeys
        //3. The features/keypoints are undistorted
        //4. The keypoints are then passed through a stereo matching algorithm which computes their depths
        //5. Thus, mvKeys contain the keypoints in terms of their x, y and mvDepth contain their depth in z coordinates
        mCurrentFrame = Frame(mImGray, imGrayRight, imLabel, imDepth, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary,
                              mK, mDistCoef, mbf, mThDepth, featureFile);
        Track();

//        cout << "Saving to disk" << endl;
//        mCurrentFrame.SaveFrameToDisk (1, &KpLog);

        UnsetThreadRunning();
        return mCurrentFrame.mTcw.clone();
    }


    cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, const double &timestamp) {
        mImGray = imRGB;
        cv::Mat imDepth = imD;

        if (mImGray.channels() == 3) {
            if (mbRGB)
                cvtColor(mImGray, mImGray, CV_RGB2GRAY);
            else
                cvtColor(mImGray, mImGray, CV_BGR2GRAY);
        } else if (mImGray.channels() == 4) {
            if (mbRGB)
                cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
            else
                cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
        }

        if ((fabs(mDepthMapFactor - 1.0f) > 1e-5) || imDepth.type() != CV_32F)
            imDepth.convertTo(imDepth, CV_32F, mDepthMapFactor);

        mCurrentFrame = Frame(mImGray, imDepth, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf,
                              mThDepth);

        Track();

        return mCurrentFrame.mTcw.clone();
    }


    cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp) {
        mImGray = im;

        if (mImGray.channels() == 3) {
            if (mbRGB)
                cvtColor(mImGray, mImGray, CV_RGB2GRAY);
            else
                cvtColor(mImGray, mImGray, CV_BGR2GRAY);
        } else if (mImGray.channels() == 4) {
            if (mbRGB)
                cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
            else
                cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
        }

        if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET)
            mCurrentFrame = Frame(mImGray, timestamp, mpIniORBextractor, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);
        else
            mCurrentFrame = Frame(mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf,
                                  mThDepth);

        Track();

        return mCurrentFrame.mTcw.clone();
    }

    cv::Mat Tracking::StitchWithMap(ORB_SLAM2::KeyFrame *kf, bool oneShotRelocalization)
    {
        // This function stitches the keyframe kf with the current map
        // oneShotRelocalization determines the stitching mode
        // True: single shot relocalization stitching - this means that we find a single anchor point and do a perspective transformation after that
        // False: progressive relocalization stitching - this means that we position every keyframe by relocalizing it in the map
        const bool isDebugMode = true;
        static bool isAnchorDetected = false; // true if we have found an anchor in the current map
        static cv::Mat refKfBaseMapTcw = cv::Mat::zeros(4,4, CV_32F); // pose of reference keyframe in base map
        static cv::Mat refKfOldMapTcw = cv::Mat::zeros(4,4, CV_32F); // pose of the reference keyframe in its old map
        static cv::Mat transformationMatrix = cv::Mat::zeros(4,4, CV_32F); // transformation matrix for transforming reference keyframe from old to the base map
        static ORB_SLAM2::KeyFrame * prevKF; // pointer to the previous/reference keyframe

        if (oneShotRelocalization == false) // stitching mode: progressive relocalization
        {
            refKfOldMapTcw = kf->GetPoseInverse(); // get the pose of the keyframe before relocalization
            bool relocalization = RelocalizeKeyFrame(kf);//relocalize the keyframe
            refKfBaseMapTcw = kf->GetPoseInverse();// get the pose of the keyframe after relocalization to calculate the transformation matrix

            if (relocalization) // if relocalization was successful, find the transformation matrix and add the keyframe to the map
            {
                transformationMatrix = refKfBaseMapTcw * refKfOldMapTcw.inv();// find the transformation matrix
                KeyFrame *newKF = CreateKF(kf, transformationMatrix);// add the keyframe to the map
                mpMapDrawer->SetCurrentCameraPose(newKF->GetPose());// update the camera pose in the viewer
                return kf->GetPose();
            }
            else
            {
                if (isDebugMode == true)
                    cout << "Relocalization failed" << endl;
                return cv::Mat();
            }
        }

        else // stitching mode : single shot relocalization
        {
            if (isAnchorDetected == false) // find the anchor point first & the transformation matrix between the old map and base map
            {
                if (isDebugMode == true)
                    cout << "Looking for an anchor keyframe ....." << endl;

                refKfOldMapTcw = kf->GetPoseInverse(); // get the pose of the keyframe before relocalization
                bool relocalization = RelocalizeKeyFrame(kf); // relocalize the keyframe in the base map
                isAnchorDetected = relocalization;
                refKfBaseMapTcw = kf->GetPoseInverse(); // get the pose of the keyframe after relocalization in the base map

                if (relocalization) // if relocalization was successful, then add the keyframe to the base map
                {
                    transformationMatrix = refKfBaseMapTcw * refKfOldMapTcw.inv();//find the transformation matrix
                    KeyFrame *newKF = CreateKF(kf, transformationMatrix);// Now add this keyframe to the map
                    prevKF = newKF;// save the pointer to the current keyframe for facilitating map-point matching
                    mpMapDrawer->SetCurrentCameraPose(newKF->GetPose());
                    return kf->GetPose();
                }
                else
                {
                    if (isDebugMode)
                        cout << "Could not relocalize the first keyframe in one shot relocalization" << endl;
                    return cv::Mat ();
                }
            }

            else // we have already found the anchor point & the transformation matrix
            {
                FindCorrespondencesBetweenKeyframes(kf, prevKF);
                transformationMatrix = refKfBaseMapTcw * refKfOldMapTcw.inv();
                KeyFrame *newKF = GeometricalKeyFrameTransformation(kf, transformationMatrix);// Now add this keyframe to the map
                prevKF = newKF;
                mpMapDrawer->SetCurrentCameraPose(newKF->GetPose());
                return kf->GetPose();
            }
        }
    }


    void Tracking::Track() {

        static int counter = 0;
        bool isDebugMode = false;
        if (isDebugMode)
            cout << "Tracking :: Track ()" << endl;

        if (mState == NO_IMAGES_YET) {
            mState = NOT_INITIALIZED;
            if (isDebugMode)
                cout << "Track () : NOT_INTIALIZED" << endl;
        }

        mLastProcessedState = mState;

        // Get Map Mutex -> Map cannot be changed
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

        if (mState == NOT_INITIALIZED)
        {
            if (isDebugMode)
                cout << "Track(): Initializing ..." << endl;
            if (mSensor == System::STEREO || mSensor == System::RGBD) {
                //1. Convert all the key points into map points
                //2. Send a signal to mapping that a new keyframe is available
                StereoInitialization();
            }
            else
                MonocularInitialization();

            mpFrameDrawer->Update(this);

            if (mState != OK)
                return;
        }
        else
        {
            // System is initialized. Track Frame.
            bool bOK;
            // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
            //Only if mapping is activated
            //SLAM MODE
            if (!mbOnlyTracking) {
                // Local Mapping is activated. This is the normal behaviour, unless
                // you explicitly activate the "only tracking" mode.
                if (mState == OK) {

                    if (isDebugMode)
                        cout << "State == OK" << endl;
                    // Local Mapping might have changed some MapPoints tracked in last frame
                    //1. Find all the map points that have been replaced
                    CheckReplacedInLastFrame();
                    if (mVelocity.empty() || mCurrentFrame.mnId < mnLastRelocFrameId + 2)
                    {
                        //For Track through Reference Keyframe, we only need MAP POINTS and their DESCRIPTORS
                        //If we don't have a velocity model or the number of frames since localization is less, then track using the keyframe
                        //1. Get the last keyframe in the map
                        //2. Find map point matches between the last keyframe and the current frame
                        //3. Optimize the matches to find pose of the current frame wrt to the previous keyframe
                        //4. Remove all outliers
                        if (isDebugMode)
                            cout << "Track(): Motion model empty so doing TrackReferenceKeyFrame" << endl;
                        bOK = TrackReferenceKeyFrame();//Updated!
                    }
                    else
                    {
                        //if we have a motion model and it has been more frames since we last relocalized, then go ahead and track using motion model
                        bOK = TrackWithMotionModel();
                        //1. Project all map points from the previous frame in the current frame
                        //2. To all those map points that can exist, try to find similar key points using descriptor distance and geometrical relationships
                        //3. Optimize the position of the current frame based on the found matches
                        //4. Return true if the matches are enough, else return false
                        if (!bOK)
                        {
                            //If we were successful, then good, otherwise go back to tracking using the reference keyframe
                            bOK = TrackReferenceKeyFrame();
                        }
                    }

                    // Since the state is OK, this means we can carry out the DiffOperation
                    if (bOK && RobustFeatureSearch)
                        DiffOperation (mCurrentFrame);
                }
                else
                {
                    //If nothing works, then relocalize!
                    if (isDebugMode)
                        cout << "Track(): Relocalization" << endl;

                    //1. Convert the frame into a bag of words
                    //2. Find neighboring keyframes in the co-visibility graph that are similar in terms of BoW
                    //3. For each similar keyframe, compare their map points with the key points of the current frame and find the closest match
                    //4. Optimize the position of the vehicle based on this and remove outlier points etc!
                    bOK = Relocalization();

                    // If we were able to localize mCurrentFrame, then carry out the diff operation
                    if (bOK == true && RobustFeatureSearch)
                        DiffOperation(mCurrentFrame);
                }
            } else {
                // Localization Mode: Local Mapping is deactivated

                if (mState == LOST) {
                    bOK = Relocalization();
                } else {
                    //NON-Visual Odometry Mode
                    if (!mbVO)//if we did tracked enough map points in the last frame
                    {
                        if (!mVelocity.empty()) {
                            bOK = TrackWithMotionModel();
                        } else {
                            bOK = TrackReferenceKeyFrame();
                        }
                    } else //VISUAL ODOMETRY MODE
                    {
                        // In last frame we tracked mainly "visual odometry" points.

                        // We compute two camera poses, one from motion model and one doing relocalization.
                        // If relocalization is sucessfull we choose that solution, otherwise we retain
                        // the "visual odometry" solution.

                        bool bOKMM = false;
                        bool bOKReloc = false;
                        vector<MapPoint *> vpMPsMM;
                        vector<bool> vbOutMM;
                        cv::Mat TcwMM;
                        if (!mVelocity.empty()) {
                            bOKMM = TrackWithMotionModel();
                            vpMPsMM = mCurrentFrame.mvpMapPoints;
                            vbOutMM = mCurrentFrame.mvbOutlier;
                            TcwMM = mCurrentFrame.mTcw.clone();
                        }
                        bOKReloc = Relocalization();

                        if (bOKMM && !bOKReloc)//If relocalization has failed, retain the visual odometry pose
                        {
                            mCurrentFrame.SetPose(TcwMM);
                            mCurrentFrame.mvpMapPoints = vpMPsMM;
                            mCurrentFrame.mvbOutlier = vbOutMM;

                            if (mbVO) {
                                for (int i = 0; i < mCurrentFrame.N; i++) {
                                    if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i]) {
                                        mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                    }
                                }
                            }
                        } else if (bOKReloc)//if we were able to relocalize, then switch off Visual Odometry mode
                        {
                            mbVO = false;
                        }

                        bOK = bOKReloc || bOKMM;
                    }
                }
            }

            mCurrentFrame.mpReferenceKF = mpReferenceKF;

            // If we have an initial estimation of the camera pose and matching. Track the local map.
            if (!mbOnlyTracking)//SLAM Mode
            {

                if (bOK) {
                    if (isDebugMode)
                        cout << "Using Track(): TrackLocalMap ()" << endl;
//                cout << "Now tracking local map" << endl;
                    bOK = TrackLocalMap();




                    if (isDebugMode)
                    {
                        if (bOK)
                            cout << "TrackLocalMap(): Successful" << endl;
                        else
                            cout << "TrackLocalMap(): Failed" << endl;
                    }

//                    if (isDebugMode)
//                        cout << "Trying TrackLocalMapWithKDTree instead";
//
//                    bOK = TrackLocalMapWithKDTree();
//                    if (isDebugMode)
//                    {
//                        if (bOK)
//                            cout << "TrackLocalMapWithKDTree(): Successful" << endl;
//                        else
//                            cout << "TrackLocalMapWithKDTree(): Failed" << endl;
//                    }
                    //Track more map points in the same area
                }

            } else//Localization Mode
            {
                // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
                // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
                // the camera we will use the local map again.
                if (bOK && !mbVO)
                    bOK = TrackLocalMap();
            }

            if (bOK) {
                mState = OK;
                if (isDebugMode)
                    cout << "Tracking:: Successful" << endl;
            }
            else
            {
                mState = LOST;
                if (isDebugMode)
                    cout << "Tracking:: Failed" << endl;

            }

            // Update drawer
            mpFrameDrawer->Update(this);

            // If tracking was good, check if we insert a keyframe
            if (bOK) {
                // Update motion model
                if (!mLastFrame.mTcw.empty()) {
                    cv::Mat LastTwc = cv::Mat::eye(4, 4, CV_32F);
                    mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0, 3).colRange(0, 3));
                    mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0, 3).col(3));
                    mVelocity = mCurrentFrame.mTcw * LastTwc;
                } else
                    mVelocity = cv::Mat();

                mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

                // Clean VO matches
                for (int i = 0; i < mCurrentFrame.N; i++) {
                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                    if (pMP)
                        if (pMP->Observations() < 1) {
                            mCurrentFrame.mvbOutlier[i] = false;
                            mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                        }
                }

                // Delete temporal MapPoints
                for (list<MapPoint *>::iterator lit = mlpTemporalPoints.begin(), lend = mlpTemporalPoints.end();
                     lit != lend; lit++) {
                    MapPoint *pMP = *lit;
                    delete pMP;
                }
                mlpTemporalPoints.clear();

                // Check if we need to insert a new keyframe
                if (NeedNewKeyFrame()) {
//                cout << "Creating a new keyframe" << endl;
                    CreateNewKeyFrame();
                }


                // We allow points with high innovation (considererd outliers by the Huber Function)
                // pass to the new keyframe, so that bundle adjustment will finally decide
                // if they are outliers or not. We don't want next frame to estimate its position
                // with those points so we discard them in the frame.
                for (int i = 0; i < mCurrentFrame.N; i++) {
                    if (mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                        mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                }
            }

            // Reset if the camera get lost soon after initialization
            // TODO: Change to 5
            if (mState == LOST) {
                cout << "Failed" << endl;
                if (mpMap->KeyFramesInMap() <= 0) {
                    cout << "Track lost soon after initialisation, reseting..." << endl;
                    mpSystem->Reset();
                    return;
                }
            }

            if (!mCurrentFrame.mpReferenceKF)
                mCurrentFrame.mpReferenceKF = mpReferenceKF;

            mLastFrame = Frame(mCurrentFrame);
        }

        // Store frame pose information to retrieve the complete camera trajectory afterwards.
        if (!mCurrentFrame.mTcw.empty()) {
            if (mCurrentFrame.mpReferenceKF != NULL) {
                cv::Mat Tcr = mCurrentFrame.mTcw * mCurrentFrame.mpReferenceKF->GetPoseInverse();
                mlRelativeFramePoses.push_back(Tcr);
                mlpReferences.push_back(mpReferenceKF);
                mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
                mlbLost.push_back(mState == LOST);
            }
        } else {
            //This can happen when tracking is lost

            if (!mlRelativeFramePoses.empty())
                mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
            mlpReferences.push_back(mlpReferences.back());
            mlFrameTimes.push_back(mlFrameTimes.back());
            mlbLost.push_back(mState == LOST);
        }
    }

    void Tracking::StereoInitialization() {
        if (mCurrentFrame.N > 500) {
            // Set Frame pose to the origin
            mCurrentFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));

            // Create the first keyframe in origin as the pose
            KeyFrame *pKFini = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB, true);

            // Insert KeyFrame in the map
            mpMap->AddKeyFrame(pKFini);

            // Create MapPoints and associate to KeyFrame
            for (int i = 0; i < mCurrentFrame.N; i++)//iterate through all the keypoints in the frame
            {
                float z = mCurrentFrame.mvDepth[i];
                if (z > 0)//only save key points that have positive depth
                {
                    if (tIsDynamicObjectRemoval) {
                        if (SemanticSegmentor::IsDynamicObject(mCurrentFrame.mvLabels[i]))
                            continue;
                    }
//
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(
                            i);//using the index i, find the x and y coordinates of the map point from the centre of the camera using camera intrinsics
//                            cout << "Created it from here" << endl;
                    MapPoint *pNewMP = new MapPoint(x3D, mCurrentFrame.mvLabels[i], pKFini, mpMap); //Create a new map point with x3D in mpMap at KF pKFini with label mvLabels[i]
//                    MapPoint *pNewMP = new MapPoint(x3D, 255, pKFini, mpMap); //Create a new map point with x3D in mpMap at KF pKFini with label mvLabels[i]
                    pNewMP->AddObservation(pKFini, i);//Note in the map point that it refers to pKFini [idx]
                    pKFini->AddMapPoint(pNewMP, i);//Add the map point to the keyframe as well (double pointers)
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i] = pNewMP;

                }
            }


            mpLocalMapper->InsertKeyFrame(pKFini);


            mLastFrame = Frame(mCurrentFrame);
            mnLastKeyFrameId = mCurrentFrame.mnId;
            mpLastKeyFrame = pKFini;

            mvpLocalKeyFrames.push_back(pKFini);
            mvpLocalMapPoints = mpMap->GetAllMapPoints();
            mpReferenceKF = pKFini;
            mCurrentFrame.mpReferenceKF = pKFini;

            mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

            mpMap->mvpKeyFrameOrigins.push_back(pKFini);

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            mState = OK;
        }
    }


    void Tracking::MonocularInitialization() {

        if (!mpInitializer) {
            // Set Reference Frame
            if (mCurrentFrame.mvKeys.size() > 100) {
                mInitialFrame = Frame(mCurrentFrame);
                mLastFrame = Frame(mCurrentFrame);
                mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
                for (size_t i = 0; i < mCurrentFrame.mvKeysUn.size(); i++)
                    mvbPrevMatched[i] = mCurrentFrame.mvKeysUn[i].pt;

                if (mpInitializer)
                    delete mpInitializer;

                mpInitializer = new Initializer(mCurrentFrame, 1.0, 200);

                fill(mvIniMatches.begin(), mvIniMatches.end(), -1);

                return;
            }
        } else {
            // Try to initialize
            if ((int) mCurrentFrame.mvKeys.size() <= 100) {
                delete mpInitializer;
                mpInitializer = static_cast<Initializer *>(NULL);
                fill(mvIniMatches.begin(), mvIniMatches.end(), -1);
                return;
            }

            // Find correspondences
            ORBmatcher matcher(0.9, true);
            int nmatches = matcher.SearchForInitialization(mInitialFrame, mCurrentFrame, mvbPrevMatched, mvIniMatches,
                                                           100);

            // Check if there are enough correspondences
            if (nmatches < 100) {
                delete mpInitializer;
                mpInitializer = static_cast<Initializer *>(NULL);
                return;
            }

            cv::Mat Rcw; // Current Camera Rotation
            cv::Mat tcw; // Current Camera Translation
            vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

            if (mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated)) {
                for (size_t i = 0, iend = mvIniMatches.size(); i < iend; i++) {
                    if (mvIniMatches[i] >= 0 && !vbTriangulated[i]) {
                        mvIniMatches[i] = -1;
                        nmatches--;
                    }
                }

                // Set Frame Poses
                mInitialFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));
                cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
                Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
                tcw.copyTo(Tcw.rowRange(0, 3).col(3));
                mCurrentFrame.SetPose(Tcw);

                CreateInitialMapMonocular();
            }
        }
    }

    void Tracking::CreateInitialMapMonocular() {
        // Create KeyFrames
        KeyFrame *pKFini = new KeyFrame(mInitialFrame, mpMap, mpKeyFrameDB);
        KeyFrame *pKFcur = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);


        pKFini->ComputeBoW();
        pKFcur->ComputeBoW();

        // Insert KFs in the map
        mpMap->AddKeyFrame(pKFini);
        mpMap->AddKeyFrame(pKFcur);

        // Create MapPoints and asscoiate to keyframes
        for (size_t i = 0; i < mvIniMatches.size(); i++) {
            if (mvIniMatches[i] < 0)
                continue;

            //Create MapPoint.
            cv::Mat worldPos(mvIniP3D[i]);

            MapPoint *pMP = new MapPoint(worldPos, pKFcur, mpMap);

            pKFini->AddMapPoint(pMP, i);
            pKFcur->AddMapPoint(pMP, mvIniMatches[i]);

            pMP->AddObservation(pKFini, i);
            pMP->AddObservation(pKFcur, mvIniMatches[i]);

            pMP->ComputeDistinctiveDescriptors();
            pMP->UpdateNormalAndDepth();

            //Fill Current Frame structure
            mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
            mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

            //Add to Map
            mpMap->AddMapPoint(pMP);
        }

        // Update Connections
        pKFini->UpdateConnections();
        pKFcur->UpdateConnections();

        // Bundle Adjustment
        cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

        Optimizer::GlobalBundleAdjustemnt(mpMap, 20);

        // Set median depth to 1
        float medianDepth = pKFini->ComputeSceneMedianDepth(2);
        float invMedianDepth = 1.0f / medianDepth;

        if (medianDepth < 0 || pKFcur->TrackedMapPoints(1) < 100) {
            cout << "Wrong initialization, reseting..." << endl;
            Reset();
            return;
        }

        // Scale initial baseline
        cv::Mat Tc2w = pKFcur->GetPose();
        Tc2w.col(3).rowRange(0, 3) = Tc2w.col(3).rowRange(0, 3) * invMedianDepth;
        pKFcur->SetPose(Tc2w);

        // Scale points
        vector<MapPoint *> vpAllMapPoints = pKFini->GetMapPointMatches();
        for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++) {
            if (vpAllMapPoints[iMP]) {
                MapPoint *pMP = vpAllMapPoints[iMP];
                pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
            }
        }

        mpLocalMapper->InsertKeyFrame(pKFini);
        mpLocalMapper->InsertKeyFrame(pKFcur);

        mCurrentFrame.SetPose(pKFcur->GetPose());
        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKFcur;

        mvpLocalKeyFrames.push_back(pKFcur);
        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints = mpMap->GetAllMapPoints();
        mpReferenceKF = pKFcur;
        mCurrentFrame.mpReferenceKF = pKFcur;

        mLastFrame = Frame(mCurrentFrame);

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        mState = OK;
    }

    void Tracking::CheckReplacedInLastFrame() {

        int counter = 0;
        for (int i = 0; i < mLastFrame.N; i++) {
            MapPoint *pMP = mLastFrame.mvpMapPoints[i];

            if (pMP) {
                MapPoint *pRep = pMP->GetReplaced();
                if (pRep) {
                    mLastFrame.mvpMapPoints[i] = pRep;
                    counter++;
                }
            }
        }


    }

    void Tracking::DiffOperation (Frame& F)
    {
        if (PerformDiffOperation == true)
        {
//            cout << "Diff operation" << endl;
            ORBmatcher matcher(1.0, MajorityVoting, true);
            matcher.SearchByKDTree(F, mpMap);
        }
    }

    bool Tracking::TrackReferenceKeyFrame() {

        const bool isDebugMode = false;
        if (isDebugMode)
            cout << "mpReferenceKeyFrame is from " << mpReferenceKF->isLoadedFromDisk << endl;

        if (isDebugMode)
            cout << "Called track reference keyframe" << endl;
        // Compute Bag of Words vector
        mCurrentFrame.ComputeBoW();

        // We perform first an ORB matching with the reference keyframe
        // If enough matches are found we setup a PnP solver
        ORBmatcher matcher(0.7, MajorityVoting, true);
        vector<MapPoint *> vpMapPointMatches;

        //Find the number of matches between the MAP POINTS in the reference keyframe and the current frame
        //The number of matches are saved in nmatches
//    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);
        int nmatches = matcher.SearchByBoW(mpReferenceKF, mCurrentFrame, vpMapPointMatches);

        if (isDebugMode)
            cout << "Number of matches found with reference keyframe = " << nmatches << endl;

        //1. Get all map point matches in the keyframe
        //2. For all map point matches in the keyframe, compare them with keypoints in the current frame and try to find those that match closest
        //3. If they match (descriptor distance and orientation), mark them and associate them to the same map point!
        //4. Return the number of matches found between the keyframe's map points and the current frame's key points




        mCurrentFrame.mvpMapPoints = vpMapPointMatches;
        mCurrentFrame.SetPose(mLastFrame.mTcw);


        if (nmatches < 15)//if the matches are less than 15, then return false
        {
            int newMatches = 0;
            if (RobustFeatureSearch)
            {
                if (PerformDiffOperation)
                    newMatches = matcher.SearchByKDTree(mCurrentFrame, mpMap);
                if (newMatches + nmatches < 15)
                {
                    cerr << "Still less than 15" << endl;
                    return false;
                }
                else
                    cout << "Worked" << endl;
            }
            else
            {
                cerr << "Still less than 15" << endl;
                return false;
            }
        }

        //The pose of the last frame is assigned to the current frame and then it is optimized

        Optimizer::PoseOptimization(&mCurrentFrame);
        //Do some fancy optimization

        // Discard outliers
        int nmatchesMap = 0;
        for (int i = 0; i < mCurrentFrame.N; i++) {
            if (mCurrentFrame.mvpMapPoints[i]) {
                if (mCurrentFrame.mvbOutlier[i]) {
                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    mCurrentFrame.mvbOutlier[i] = false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                } else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                    nmatchesMap++;
            }
        }

        return nmatchesMap >= 10;
    }

    void Tracking::UpdateLastFrame() {
        // Update pose according to reference keyframe
        KeyFrame *pRef = mLastFrame.mpReferenceKF;
        cv::Mat Tlr = mlRelativeFramePoses.back();

        mLastFrame.SetPose(Tlr * pRef->GetPose());

        if (mnLastKeyFrameId == mLastFrame.mnId || mSensor == System::MONOCULAR || !mbOnlyTracking)
            return;

        // Create "visual odometry" MapPoints
        // We sort points according to their measured depth by the stereo/RGB-D sensor
        vector<pair<float, int> > vDepthIdx;
        vDepthIdx.reserve(mLastFrame.N);
        for (int i = 0; i < mLastFrame.N; i++) {
            float z = mLastFrame.mvDepth[i];
            if (z > 0) {
                vDepthIdx.push_back(make_pair(z, i));
            }
        }

        if (vDepthIdx.empty())
            return;

        sort(vDepthIdx.begin(), vDepthIdx.end());

        // We insert all close points (depth<mThDepth)
        // If less than 100 close points, we insert the 100 closest ones.
        int nPoints = 0;
        for (size_t j = 0; j < vDepthIdx.size(); j++) {
            int i = vDepthIdx[j].second;

            bool bCreateNew = false;

            MapPoint *pMP = mLastFrame.mvpMapPoints[i];
            if (!pMP)
                bCreateNew = true;
            else if (pMP->Observations() < 1) {
                bCreateNew = true;
            }

            if (bCreateNew) {
                cv::Mat x3D = mLastFrame.UnprojectStereo(i);
                MapPoint *pNewMP = new MapPoint(x3D, mpMap, &mLastFrame, i);

                mLastFrame.mvpMapPoints[i] = pNewMP;

                mlpTemporalPoints.push_back(pNewMP);
                nPoints++;
            } else {
                nPoints++;
            }

            if (vDepthIdx[j].first > mThDepth && nPoints > 100)
                break;
        }
    }

    bool Tracking::TrackWithMotionModel() {
        ORBmatcher matcher(0.9, MajorityVoting, true);

        // Update last frame pose according to its reference keyframe
        // Create "visual odometry" points if in Localization Mode
        UpdateLastFrame();

        mCurrentFrame.SetPose(mVelocity * mLastFrame.mTcw);

        fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint *>(NULL));

        // Project points seen in previous frame
        int th;
        if (mSensor != System::STEREO)
            th = 15;
        else
            th = 7;
        //Find number of matches between current frame and last frame (NOT KEYFRAME) using search by projection
        int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, th, mSensor == System::MONOCULAR);
        //1. For this, search through all the map points found in the last frame and project them into the current frame
        //2. Only consider those points that can actually exist in the current frame
        //3. Now, after applying projection, apply a radius filter around that point and try to find similar keypoints in the current frame
        //4. Use descriptor distance to find these and then return the number of found matches!


        // If few matches, uses a wider window search
        // Use a wider search to find more matches
        if (nmatches < 20) {
            fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint *>(NULL));
            nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2 * th, mSensor == System::MONOCULAR);
        }

        if (nmatches < 20)
            return false;//If we didn't find enough matches, then return a Failure flag!

        // Optimize frame pose with all matches
        Optimizer::PoseOptimization(&mCurrentFrame);
        //Do that sophisticated pose optimization again

        // Discard outliers
        int nmatchesMap = 0;
        for (int i = 0; i < mCurrentFrame.N; i++) {
            if (mCurrentFrame.mvpMapPoints[i]) {
                if (mCurrentFrame.mvbOutlier[i]) {
                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    mCurrentFrame.mvbOutlier[i] = false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                } else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                    nmatchesMap++;
            }
        }

        if (mbOnlyTracking) {
            mbVO = nmatchesMap < 10;
            return nmatches > 20;
        }

        return nmatchesMap >= 10;
    }

    bool Tracking::TrackLocalMapWithKDTree ()
    {
        const bool isDebugMode = false;
        if (isDebugMode)
            cout << "Tracking::TrackLocalMapWithKDTree" << endl;
        // We have an estimation of the camera pose and some map points tracked in the frame.
        // We retrieve the neighboring map-points of all features in the current frame using our KD tree structure
        vector <MapPoint*> vpMapPointMatches; // vector to store map-point matches for current frame
        ORBmatcher matcher = ORBmatcher (0.6);
        int matches = matcher.SearchByKDTree(mCurrentFrame, mpMap, vpMapPointMatches, 5.0);

        if (isDebugMode)
            cout << "TrackLocalMapWithKDTree: The number of matches = " << matches << endl;

        // Going to do pose optimization now
        Optimizer::PoseOptimization(&mCurrentFrame);

        if (isDebugMode)
            cout << "TrackLocalMapWithKDTree: Done with pose optimization" << endl;


        mnMatchesInliers = 0;

        // Update MapPoints Statistics
        for (int i = 0; i < mCurrentFrame.N; i++) {
            if (mCurrentFrame.mvpMapPoints[i]) {
                if (!mCurrentFrame.mvbOutlier[i]) {
                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                    if (!mbOnlyTracking) {
                        if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                            mnMatchesInliers++;
                    } else
                        mnMatchesInliers++;
                } else if (mSensor == System::STEREO)
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);

            }
        }

        // Decide if the tracking was succesful
        // More restrictive if there was a relocalization recently
        if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && mnMatchesInliers < 50) {
            if (isDebugMode)
                cout << "TrackLocalMapWithKDTree(): Failed because mnMatchInliers = " << mnMatchesInliers << " less than 50" << endl;
            return false;
        }

        if (mnMatchesInliers < 30)
        {
            if (isDebugMode)
                cout << "TrackLocalMapWithKDTree(): Failed because mnMatchInliers = " << mnMatchesInliers << " are less than 30" << endl;
            return false;
        }
        else {
            if (isDebugMode)
                cout << "TrackLocalMapWithKDTree(): Successful with mnMatchesInliers = " << mnMatchesInliers << endl;
            return true;
        }
    }

    bool Tracking::TrackLocalMap() {
        const bool isDebugMode = false;
        // We have an estimation of the camera pose and some map points tracked in the frame.
        // We retrieve the local map and try to find matches to points in the local map.

        // Get the local map (keyframes and map-point) for the current frame
        // Populate mvpLocalKeyFrames and mvpLocalMapPoints
        // These respond to all the key-frames and map-points that might be present in the vicinity of the current frame
        UpdateLocalMap();

        // Using mvpLocalMapPoints and mvpLocalKeyFrames, match all map-points in the local map to the current frame
        SearchLocalPoints();
//
        if (PerformDiffOperation && RobustFeatureSearch) {
//            cout << "Going to do KD-tree now" << endl;
            ORBmatcher matcher(0.7, true);
            int matches = matcher.SearchByKDTree(mCurrentFrame, mpMap);
//            cout << "Number of new matches = " << matches << endl;
        }

        // Optimize Pose

        // Using the set of new matches, further optimize the pose of the camera
        Optimizer::PoseOptimization(&mCurrentFrame);


        mnMatchesInliers = 0;

        // Update MapPoints Statistics
        for (int i = 0; i < mCurrentFrame.N; i++) {
            if (mCurrentFrame.mvpMapPoints[i]) {
                if (!mCurrentFrame.mvbOutlier[i]) {
                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                    if (!mbOnlyTracking) {
                        if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                            mnMatchesInliers++;
                    } else
                        mnMatchesInliers++;
                } else if (mSensor == System::STEREO)
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);

            }
        }

        // Decide if the tracking was succesful
        // More restrictive if there was a relocalization recently
//        if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && mnMatchesInliers < 50) {
        if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && mnMatchesInliers < 30) {
            if (isDebugMode)
                cout << "TrackLocalMap(): Failed because mnMatchInliers = " << mnMatchesInliers << " less than 50" << endl;
            return false;
        }

        if (mnMatchesInliers < 30) {
            if (isDebugMode)
                cout << "TrackLocalMap(): Failed because mnMatchInliers = " << mnMatchesInliers << " are less than 30" << endl;
            return false;
        }
        else {
            if (isDebugMode)
                cout << "TrackLocalMap(): Successful with mnMatchesInliers = " << mnMatchesInliers << endl;
            return true;
        }
    }


    bool Tracking::NeedNewKeyFrame() {
        if (mbOnlyTracking)
            return false;

        // If Local Mapping is freezed by a Loop Closure do not insert keyframes
        if (mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
            return false;

        const int nKFs = mpMap->KeyFramesInMap();

        // Do not insert keyframes if not enough frames have passed from last relocalisation
        if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && nKFs > mMaxFrames)
            return false;

        // Tracked MapPoints in the reference keyframe
        int nMinObs = 3;
        if (nKFs <= 2)
            nMinObs = 2;
        int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

        // Local Mapping accept keyframes?
        bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

        // Check how many "close" points are being tracked and how many could be potentially created.
        int nNonTrackedClose = 0;
        int nTrackedClose = 0;
        if (mSensor != System::MONOCULAR) {
            for (int i = 0; i < mCurrentFrame.N; i++) {
                if (mCurrentFrame.mvDepth[i] > 0 && mCurrentFrame.mvDepth[i] < mThDepth) {
                    if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                        nTrackedClose++;
                    else
                        nNonTrackedClose++;
                }
            }
        }

        bool bNeedToInsertClose = (nTrackedClose < 100) && (nNonTrackedClose > 70);

        // Thresholds
        float thRefRatio = 0.75f;
        if (nKFs < 2)
            thRefRatio = 0.4f;

        if (mSensor == System::MONOCULAR)
            thRefRatio = 0.9f;

        // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
        const bool c1a = mCurrentFrame.mnId >= mnLastKeyFrameId + mMaxFrames;
        // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
        const bool c1b = (mCurrentFrame.mnId >= mnLastKeyFrameId + mMinFrames && bLocalMappingIdle);
        //Condition 1c: tracking is weak
        const bool c1c = mSensor != System::MONOCULAR && (mnMatchesInliers < nRefMatches * 0.25 || bNeedToInsertClose);
        // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
        const bool c2 = ((mnMatchesInliers < nRefMatches * thRefRatio || bNeedToInsertClose) && mnMatchesInliers > 15);

        if ((c1a || c1b || c1c) && c2) {
            // If the mapping accepts keyframes, insert keyframe.
            // Otherwise send a signal to interrupt BA
            if (bLocalMappingIdle) {
                return true;
            } else {
                mpLocalMapper->InterruptBA();
                if (mSensor != System::MONOCULAR) {
                    if (mpLocalMapper->KeyframesInQueue() < 3)
                        return true;
                    else
                        return false;
                } else
                    return false;
            }
        } else
            return false;
    }


    KeyFrame *Tracking::CreateKF(KeyFrame *kf, cv::Mat transformationMatrix) {
//        if (!mpLocalMapper->SetNotStop(true)) {
//            cerr << "Create KF returning null" << endl;
//            return (static_cast<KeyFrame *> (NULL));
//        }

        KeyFrame *pKF = new KeyFrame(kf, mpMap, mpKeyFrameDB, transformationMatrix);
        pKF->SetORBvocabulary(mpORBVocabulary);
        pKF->ReconstructKeyFrame(mpKeyFrameDB, mpMap);

        mpReferenceKF = pKF;
        mCurrentFrame.mpReferenceKF = pKF;

        mpLocalMapper->InsertKeyFrame(pKF);

        mpLocalMapper->SetNotStop(false);

        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKF;

        return pKF;
    }



    KeyFrame *Tracking::GeometricalKeyFrameTransformation(KeyFrame *kf, cv::Mat transformationMatrix) {
//        if (!mpLocalMapper->SetNotStop(true)) {
//            cerr << "Create KF returning null" << endl;
//            return (static_cast<KeyFrame *> (NULL));
//        }

        cout << "Making a new keyframe" << endl;
        KeyFrame *pKF = new KeyFrame(kf, mpMap, mpKeyFrameDB, true, transformationMatrix);
        pKF->SetORBvocabulary(mpORBVocabulary);
        pKF->ReconstructKeyFrame(mpKeyFrameDB, mpMap);

        mpReferenceKF = pKF;
        mCurrentFrame.mpReferenceKF = pKF;

        mpLocalMapper->InsertKeyFrame(pKF);

        mpLocalMapper->SetNotStop(false);

        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKF;

        return pKF;
    }


    void Tracking::CreateNewKeyFrame() {
        if (!mpLocalMapper->SetNotStop(true))
            return;

//        cout << "Creating a new keyframe" << endl;
        KeyFrame *pKF = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB, true);

        mpReferenceKF = pKF;
        mCurrentFrame.mpReferenceKF = pKF;

        if (mSensor != System::MONOCULAR) {
            mCurrentFrame.UpdatePoseMatrices();

            // We sort points by the measured depth by the stereo/RGBD sensor.
            // We create all those MapPoints whose depth < mThDepth.
            // If there are less than 100 close points we create the 100 closest.
            vector<pair<float, int> > vDepthIdx;
            vDepthIdx.reserve(mCurrentFrame.N);
            for (int i = 0; i < mCurrentFrame.N; i++) {
                float z = mCurrentFrame.mvDepth[i];
//
//                //TODO: skipping dynamic map-points
                if (tIsDynamicObjectRemoval == true) {
                    if (SemanticSegmentor::IsDynamicObject(mCurrentFrame.mvLabels[i]))
                        continue;
                }

                if (z > 0) {
                    vDepthIdx.push_back(make_pair(z, i));
                }
            }

            if (!vDepthIdx.empty()) {
                sort(vDepthIdx.begin(), vDepthIdx.end());

                int nPoints = 0;
                for (size_t j = 0; j < vDepthIdx.size(); j++) {
                    int i = vDepthIdx[j].second;

                    bool bCreateNew = false;

                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                    if (!pMP)
                        bCreateNew = true;
                    else if (pMP->Observations() < 1) {
                        bCreateNew = true;
                        mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    }

                    if (bCreateNew) {
                        cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);

                        unsigned char label;
//                        if (pKF->GetmnId() % 700 == 0)
                            label = mCurrentFrame.mvLabels[i];
//                        else
//                            label = 255;

                        MapPoint *pNewMP = new MapPoint(x3D, label, pKF, mpMap);
//                        cout << "Setting label to 255" << endl;
//                        MapPoint *pNewMP = new MapPoint(x3D, 255, pKF, mpMap);
                        pNewMP->AddObservation(pKF, i);
                        pKF->AddMapPoint(pNewMP, i);
                        pNewMP->ComputeDistinctiveDescriptors();
                        pNewMP->UpdateNormalAndDepth();
                        mpMap->AddMapPoint(pNewMP);

                        mCurrentFrame.mvpMapPoints[i] = pNewMP;
                        nPoints++;
                    } else {
                        nPoints++;
                    }

                    if (vDepthIdx[j].first > mThDepth && nPoints > 100)
                        break;
                }
            }
        }

        mpLocalMapper->InsertKeyFrame(pKF);

        mpLocalMapper->SetNotStop(false);

        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKF;
    }

    void Tracking::SearchLocalPoints() {
        // Do not search map points already matched

        // Match all map-points in the local map to the current frame
        for (vector<MapPoint *>::iterator vit = mCurrentFrame.mvpMapPoints.begin(), vend = mCurrentFrame.mvpMapPoints.end();
             vit != vend; vit++) {
            MapPoint *pMP = *vit;
            if (pMP) {
                if (pMP->isBad()) {
                    *vit = static_cast<MapPoint *>(NULL);
                } else {
                    pMP->IncreaseVisible();
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    pMP->mbTrackInView = false;
                }
            }
        }

        int nToMatch = 0;

        // Project points in frame and check its visibility
        for (vector<MapPoint *>::iterator vit = mvpLocalMapPoints.begin(), vend = mvpLocalMapPoints.end();
             vit != vend; vit++) {
            MapPoint *pMP = *vit;
            if (pMP->mnLastFrameSeen == mCurrentFrame.mnId)
                continue;
            if (pMP->isBad())
                continue;
            // Project (this fills MapPoint variables for matching)
            if (mCurrentFrame.isInFrustum(pMP, 0.5)) {
                pMP->IncreaseVisible();
                nToMatch++;
            }
        }

        if (nToMatch > 0) {
            ORBmatcher matcher(0.8);
            int th = 1;
            if (mSensor == System::RGBD)
                th = 3;
            // If the camera has been relocalised recently, perform a coarser search
            if (mCurrentFrame.mnId < mnLastRelocFrameId + 2)
                th = 5;
            matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th);
        }
    }

    void Tracking::UpdateLocalMap() {
        // This is for visualization
        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        // Update
        UpdateLocalKeyFrames(); // get all those keyframes that might be in the vicinity of the current frame
        // By vicinity, we mean all those keyframes that share one or more map-points with the current frame
        // We also mean all those keyframes that are neighbors of first-neighbors
        UpdateLocalPoints(); // get the map-points belong to local keyframes in the map
    }

    void Tracking::UpdateLocalPoints() {
        mvpLocalMapPoints.clear();

        for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end();
             itKF != itEndKF; itKF++) {
            KeyFrame *pKF = *itKF;
            const vector<MapPoint *> vpMPs = pKF->GetMapPointMatches();

            for (vector<MapPoint *>::const_iterator itMP = vpMPs.begin(), itEndMP = vpMPs.end();
                 itMP != itEndMP; itMP++) {
                MapPoint *pMP = *itMP;
                if (!pMP)
                    continue;
                if (pMP->mnTrackReferenceForFrame == mCurrentFrame.mnId)
                    continue;
                if (!pMP->isBad()) {
                    mvpLocalMapPoints.push_back(pMP);
                    pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                }
            }
        }
    }


    void Tracking::UpdateLocalKeyFrames() {
        // Each map point vote for the keyframes in which it has been observed

        map<KeyFrame *, int> keyframeCounter; // record the number of map-point observations for this frame wrt to all keyframes
        for (int i = 0; i < mCurrentFrame.N; i++) {
            if (mCurrentFrame.mvpMapPoints[i]) {
                MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                if (!pMP->isBad()) {
                    const map<KeyFrame *, size_t> observations = pMP->GetObservations();
                    for (map<KeyFrame *, size_t>::const_iterator it = observations.begin(), itend = observations.end();
                         it != itend; it++)
                        keyframeCounter[it->first]++;
                } else {
                    mCurrentFrame.mvpMapPoints[i] = NULL;
                }
            }
        }

        // return if we did not find any correspondences to other keyframes
        if (keyframeCounter.empty())
            return;

        int max = 0;
        KeyFrame *pKFmax = static_cast<KeyFrame *>(NULL);

        mvpLocalKeyFrames.clear();
        mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

        // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
        for (map<KeyFrame *, int>::const_iterator it = keyframeCounter.begin(), itEnd = keyframeCounter.end();
             it != itEnd; it++) {
            KeyFrame *pKF = it->first;

            if (pKF->isBad())
                continue;

            if (it->second > max) {
                max = it->second;
                pKFmax = pKF;
            }

            mvpLocalKeyFrames.push_back(it->first);
            pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
        }


        // Include also some not-already-included keyframes that are neighbors to already-included keyframes
        for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end();
             itKF != itEndKF; itKF++) {
            // Limit the number of keyframes
            if (mvpLocalKeyFrames.size() > 80)
                break;

            KeyFrame *pKF = *itKF;

            const vector<KeyFrame *> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

            for (vector<KeyFrame *>::const_iterator itNeighKF = vNeighs.begin(), itEndNeighKF = vNeighs.end();
                 itNeighKF != itEndNeighKF; itNeighKF++) {
                KeyFrame *pNeighKF = *itNeighKF;
                if (!pNeighKF->isBad()) {
                    if (pNeighKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
                        mvpLocalKeyFrames.push_back(pNeighKF);
                        pNeighKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                        break;
                    }
                }
            }

            const set<KeyFrame *> spChilds = pKF->GetChilds();
            for (set<KeyFrame *>::const_iterator sit = spChilds.begin(), send = spChilds.end(); sit != send; sit++) {
                KeyFrame *pChildKF = *sit;
                if (!pChildKF->isBad()) {
                    if (pChildKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
                        mvpLocalKeyFrames.push_back(pChildKF);
                        pChildKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                        break;
                    }
                }
            }

            KeyFrame *pParent = pKF->GetParent();
            if (pParent) {
                if (pParent->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
                    mvpLocalKeyFrames.push_back(pParent);
                    pParent->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                    break;
                }
            }

        }

        if (pKFmax) {
            mpReferenceKF = pKFmax;
            mCurrentFrame.mpReferenceKF = mpReferenceKF;
        }
    }



    bool Tracking::FindCorrespondencesBetweenKeyframes (ORB_SLAM2::KeyFrame *kf, ORB_SLAM2::KeyFrame* refKf)
    {
        const bool isDebugMode = false;
        // Perform an ORB matching to get all matching map-points with the reference keyframe
        ORBmatcher matcher(0.75, true);

        // Vector to store all the map-points
        vector<MapPoint *> vvpMapPointMatches;

        // Store the previous keyframe that we want to compare the current one with and use it as a reference keyframe
        KeyFrame *pKF = refKf;
        if (pKF->isBad())
        {
            if (isDebugMode)
                cout << "Reference keyframe " << pKF->GetmnId() << " is bad" << endl;
        }
        else
        {
            int nmatches = matcher.SearchByBoWForMapPoints(pKF, kf, vvpMapPointMatches);
            // Print the number of matches with the reference keyframe
            if (isDebugMode)
            {
                cout << endl << "Reference keyframe = " << pKF->GetmnId() << ", matches / map points = " << nmatches << " / "
                     << kf->GetIndexedMapPoints().size();
                cout << " = " << (nmatches * 100.0) / kf->GetIndexedMapPoints().size() << "%" << endl;
            }
            // If the number of matches are less than 15, skip this candidate because we cannot estimate pose from this
            if (nmatches < 15)
            {
                if (isDebugMode)
                    cout << "DISCARDED " << pKF->mnId << " for less than 15 matches" << endl;
                return false;
            }
        }


        // Now going through all the map-points
        // Add those map-points already present in base map AS IS
        // For the rest, add them to duplicate map point's vector

        // Make a copy of the whole map point vector
        kf->CreateDuplicateMapPoints();
        kf->ClearMapPointMatches();
        if (isDebugMode)
            cout << "Number of all map-points or vvpMapPointsMatches with Null elements = " << vvpMapPointMatches.size() << endl;

        int inliers = 0;
        int outliers = 0;
        for (int mpCounter = 0; mpCounter < vvpMapPointMatches.size(); mpCounter++)
        {
            // Get the next map point in the matched map points
            ORB_SLAM2::MapPoint *nextMP = vvpMapPointMatches.at(mpCounter);


            // Check if there exists a map-point match and the map-point is not bad
            if ( ( nextMP != NULL) && (nextMP->isBad() == false ) )
            {
                // Map point match exists, so just add it as it is
                if (isDebugMode)
                {
//                    if ( ( kf->mvpMapPoints[mpCounter] == NULL ) || (kf->mvpMapPoints[mpCounter]->isBad() == true) )
//                    {

                    cout << mpCounter << ": Found a map point match with [KF, MP]: "<< pKF->GetmnId() << ", " << nextMP->mnId << "]" << endl;
//                    }
//                    else
//                        cout << mpCounter << ": Should not even come here" << endl;
//                        cout << mpCounter << ": Replacing [KF, MP] : [" << kf->GetmnId() << ", " << kf->mvpMapPoints[mpCounter]->mnId << "]  with [" << pKF->GetmnId() << ", " << nextMP->mnId << "]" << endl;
                }

                kf->mvpMapPoints [mpCounter] = nextMP;
                kf->dupMvpMPs [mpCounter] = static_cast<MapPoint*> (NULL);
                inliers++;
            }
            else
            {
                //This is a new map-point, so add it as it is instead of replacing it
                if (isDebugMode) {
                    cout << mpCounter << ": No map point match here" << endl;
//                    if ((kf->mvpMapPoints[mpCounter] != NULL) && (kf->mvpMapPoints[mpCounter]->isBad() == false))
//                        cout << mpCounter << ": No match for [KF, MP] : [" << kf->mnId << ", " << kf->mvpMapPoints[mpCounter]->mnId << " ]" << endl;
//                    else
//                    {
//                        cout << mpCounter << ": No match for a NULL map point" << endl;
//                    }

                }

                kf->mvpMapPoints [mpCounter] = static_cast<MapPoint*>(NULL);
                outliers++;

            }
        }

        if (isDebugMode) {
            cout << "Inliers = " << inliers << ", outliers = " << outliers << endl;
            cout << "Indexed map points = " << kf->GetIndexedMapPoints().size() << endl;
            cout << "Outlier map points = " << kf->GetOutliers().size() << endl;
        }
    }


    bool Tracking::
    RelocalizeKeyFrame(ORB_SLAM2::KeyFrame *kf) {

        const bool isDebugMode = false;

        {
            // Reconstruct the Bag of Words Vector
            //1. Compute the BoW Vector for the current frame
            //  i. Convert the descriptor matrix into a vector of descriptors
            //  ii. Transform the descriptor vector into mBowVec and mFeatVec using 4 levels of vocabulary
            //      a. Clear the mBowVec and mFeatVec before starting
            //      b. Iterate through all features for the current frame
            //      c. For each feature, call transform function with feature, wordID, nodeID and orb level
            //          A. The wordID is the ID for the visual word
            //          B. The nodeID is the ID of the node that contains the visual word
            //          C. The feature is categorized into the vocabulary tree based on the Hamming distance between words in different nodes
            //      d. If a word exists for the feature, add it to the BoW vector along with its weight in the image
            //      e. If a word exists for the feature, add it to the FeatVect (direct index) using its NodeID, and its index
            //  So at the end, we have a BoWVect which is a sparse vector for BoVW matching AND we have a FeatVect (a direct index)
            //  We will use the FeatVect for finding correspondences faster
            kf->ReconstructBoW();
            // Find relocalization candidates using the common visual words from the BoVW vectors of all keyframes
            //1. Detect relocalization candidates for mCurrentFrame according to the BoVW model and all of them in a Keyframe* vector
            //1. Get the BoWVector for the current frame and iterate through all of its words
            //   The BoWVector is indexed by words and their associated weights
            //2. For each word, use the mvInvertedFile to find KF's that contain that specific word
            //3. For all those keyframes, put them in a vector as candidates, mark them as relocalization candidates for the current frame and count the number of shared words
            //4. Return if we have no keyframe relocalization candidates
            //5. Iterate through all keyframe relocalization candidates and find the maximum number of shared words between current frame and any relocalization keyframe
            //6. After that start computing their similarity scores ONLY for those that have greater similar words than 80%
            //  i. For each candidate, form an std::pair withe keyframe and BoVW SCORE while eliminating those kf's
            //     BoVW SCORE is calculated using the two BoWVec vectors which consist of visual words and their visibility scores
            //     So for each keyframe, we have reloc_query, reloc_words and reloc_score
            //7. Now iterate through the whole list of keyframes & their scores that share enough words
            //8. For each keyframe, query its covisibilty graph and get upto 10 keyframes neighbors for it
            //  i. Now using these neighbors, build an accumulative score for each keyframe by adding the scores of its neighbors
            //  ii. If a neighbor has more score than the keyframe, replace it and use the neighbor instead in the acc score keyframe pair
            //9. Using the 0.75 * bestScore, filter out the keyframes and return the rest as relocalization candidates
            vector<ORB_SLAM2::KeyFrame *> vpCandidateKFs = mpKeyFrameDB->DetectOverlappingKeyFrameCandidates(kf);
            // Return early if we did not find any relocalization candidates
//            vpCandidateKFs = mpMap->GetAllKeyFrames();
            if (vpCandidateKFs.empty()) {
                if (isDebugMode)
                    cout << "Could not find a single vp candidate kf" << endl;
                return false;
            } else {
                if (isDebugMode) {
                    cout << "RC = " << vpCandidateKFs.size() << " (";
                    for (KeyFrame *kf : vpCandidateKFs) {
                        cout << kf->mnId << " - ";
                    }
                    cout << "), ";
                }
            }



            // Find map-point matches between current keyframe and all relocalization candidates

            const int nKFs = vpCandidateKFs.size();
            // We perform first an ORB matching with each candidate
            ORBmatcher matcher(0.75, true);
            // If enough matches are found we setup a PnP solver
            vector<PnPsolver *> vpPnPsolvers;
            vpPnPsolvers.resize(nKFs);
            vector<vector<MapPoint *> > vvpMapPointMatches;
            vvpMapPointMatches.resize(nKFs);
            vector<bool> vbDiscarded;
            vbDiscarded.resize(nKFs);
            int nCandidates = 0;

            if (isDebugMode)
                cout << "Before the loop" << endl;
            for (int i = 0; i < nKFs; i++) {
                KeyFrame *pKF = vpCandidateKFs[i];
//                if (isDebugMode)
//                    cout << "Checking if the keyframe is bad" << endl;
                if (pKF->isBad()) {
//                    if (isDebugMode)
//                        cout << "Stuck because it was bad" << endl;
                    vbDiscarded[i] = true;
                } else {
//                    if (isDebugMode)
//                        cout << "Stuck because it was not bad" << endl;

                    int nmatches = matcher.SearchByBoWForMapPoints(pKF, kf, vvpMapPointMatches[i]);
                    if (isDebugMode) {
                        cout << endl << "RC NO [" << pKF->mnId << "], matches / map points = " << nmatches << " / "
                             << kf->GetIndexedMapPoints().size();
                        cout << " = " << (nmatches * 100.0) / kf->GetIndexedMapPoints().size() << "% -- ";
                    }

//                    cout << "Trying kd tree matching" << endl;

                    int newMatches = 0;
                    if (nmatches < 15)
                        newMatches = matcher.SearchByKDTree(kf, mpMap, vvpMapPointMatches[i]);
//
                    cout << "Number of new matches = " << newMatches << endl;

//                    cin.get();
                    // If the number of matches are less than 15, skip this candidate because we cannot estimate pose from this
                    if (nmatches + newMatches < 15) {
                        vbDiscarded[i] = true;
                        if (isDebugMode)
                            cout << "DISCARDED " << pKF->mnId << " for less than 15 matches" << endl;
                        continue;
                    } else {
                        // We have more than 15 matches, so set up a PnP matcher
                        PnPsolver *pSolver = new PnPsolver(kf, vvpMapPointMatches[i]);
                        // We reduce the number required inliers to 5 and increase the max iterations to 600
                        pSolver->SetRansacParameters(0.99, 5, 600, 4, 0.5, 5.991);
                        // ( probability, minInliers, maxIterations, minSet, epsilon, th2)
                        vpPnPsolvers[i] = pSolver;
                        nCandidates++;
                    }
                }
            }


            // Predict pose of the keyframe
            // Now that we have an initial pose, set up RANSAC to remove outliers and further optimize the camera pose
            bool bMatch = false;
            ORBmatcher matcher2(0.9, true);

            kf->CreateDuplicateMapPoints();
            cv::Mat poses[nKFs];
            int maxInliers = 0;
            int maxInliersIndex = 0;
            vector<bool> inlierArray[nKFs];
            while (nCandidates > 0 && !bMatch) {
                kf->ClearMapPointMatches();
                for (int i = 0; i < nKFs; i++) {
                    KeyFrame *rKF = vpCandidateKFs[i];
                    if (isDebugMode) {
                        cout << endl << "KF PE [" << rKF->mnId << " ] = ";
                    }
                    // Skip the the relocalization candidates that have been discarded
                    if (vbDiscarded[i]) {
                        if (isDebugMode)
                            cout << "Discard" << endl;
                        continue;
                    }
                    // Perform 5 Ransac Iterations
                    vector<bool> vbInliers;
                    int nInliers;
                    bool bNoMore;
                    PnPsolver *pSolver = vpPnPsolvers[i];
                    cv::Mat Tcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers);
                    poses[i] = Tcw;
                    inlierArray[i] = vbInliers;


                    // If Ransac reachs max. iterations discard keyframe
                    if (bNoMore) {
                        vbDiscarded[i] = true;
                        nCandidates--;
                        if (isDebugMode)
                            cout << "RANSAC discard, ";
                    }


                    if (i == 0)//first keyframe
                    {
                        maxInliers = vbInliers.size();
                        maxInliersIndex = i;
                    } else {
                        if (maxInliers < vbInliers.size()) {
                            maxInliers = vbInliers.size();
                            maxInliersIndex = i;
                        }
                    }

                    // If we have been successful in estimating the pose of the camera
                    // Save the map-point matches and the outliers in the keyframe
                    if (!Tcw.empty()) {
                        kf->SetPose(Tcw);//Copy the camera pose to the keyframe
                        set<MapPoint *> sFound;
                        const int np = vbInliers.size();

                        int counter = 0;
                        for (int j = 0; j < np; j++) {
                            if (vbInliers[j]) {
                                kf->mvpMapPoints[j] = vvpMapPointMatches[i][j];
                                kf->dupMvpMPs[j] = static_cast<MapPoint *>(NULL);
                                sFound.insert(vvpMapPointMatches[i][j]);
                                counter++;
                            } else {
                                kf->mvpMapPoints[j] = NULL;
                            }

                        }


//                        kf->PrintAllMapPoints2DCoordinates ();
//                        cout << "Running RotationPoseOptimization ()" << endl;
//                        int a1 = Optimizer::RotationPoseOptimization(kf, rKF);
//                        int a2 = Optimizer::RotationPoseOptimization(kf, rKF);
//                        int a3 = Optimizer::RotationPoseOptimization(kf, rKF);
//                        cout << "Afterwards" << endl;
//                        cout << a1 << endl;
//                        cout << a2 << endl;
//                        cout << a3 << endl;

//                        poses[i] = kf->GetPose();
//                        kf->PrintAllMapPoints2DCoordinates ();



                        bMatch = true;

                        for (int io = 0; io < kf->GetIndexedMapPoints().size(); io++)
                            if (kf->mvbOutlier[io])
                                kf->mvpMapPoints[io] = static_cast<MapPoint *>(NULL);

                    } else;
//                        cout << "No pose" << endl;
                }
            }

            if (!bMatch) {
                return false;
            } else {

                if (isDebugMode)
                    cout << endl << "----------" << endl << "Final pose index = " << vpCandidateKFs[maxInliersIndex]->mnId << endl;

                kf->SetPose(poses[maxInliersIndex]);

                int np = inlierArray[maxInliersIndex].size();

                for (int j = 0; j < np; j++) {
                    if (inlierArray[maxInliersIndex][j]) {
                        kf->mvpMapPoints[j] = vvpMapPointMatches[maxInliersIndex][j];
                        if (kf->mvpMapPoints[j]->GetReferenceKeyFrame()->mnId ==
                            kf->mvpMapPoints[j]->GetReferenceKeyFrame()->nNextId)
                            cerr << "pRefKF for map point = " << kf->mvpMapPoints[j]->GetReferenceKeyFrame()->mnId
                                 << endl;
                        kf->dupMvpMPs[j] = static_cast<MapPoint *>(NULL);
//                        sFound.insert(vvpMapPointMatches[i][j]);
//                        counter++;
                    } else {
                        kf->mvpMapPoints[j] = NULL;
                    }
                }



                for (int io = 0; io < kf->GetIndexedMapPoints().size(); io++)
                    if (kf->mvbOutlier[io])
                        kf->mvpMapPoints[io] = static_cast<MapPoint *>(NULL);


                if (isDebugMode)
                    cout << "Inliers / map points = " << kf->GetIndexedMapPoints().size() << " / "
                         << kf->GetIndexedMapPoints().size() + kf->GetOutliers().size() << endl;

                return true;
            }

        }
    }

    bool Tracking::Relocalization() {

        const bool isDebugMode = true;

        if (isDebugMode)
            cout << "Tracking::Relocalization ()" << endl;

        //1. Compute the BoW Vector for the current frame
        //  i. Convert the descriptor matrix into a vector of descriptors
        //  ii. Transform the descriptor vector into mBowVec and mFeatVec using 4 levels of vocabulary
        //      a. Clear the mBowVec and mFeatVec before starting
        //      b. Iterate through all features for the current frame
        //      c. For each feature, call transform function with feature, wordID, nodeID and orb level
        //          A. The wordID is the ID for the visual word
        //          B. The nodeID is the ID of the node that contains the visual word
        //          C. The feature is categorized into the vocabulary tree based on the Hamming distance between words in different nodes
        //      d. If a word exists for the feature, add it to the BoW vector along with its weight in the image
        //      e. If a word exists for the feature, add it to the FeatVect (direct index) using its NodeID, and its index
        //  So at the end, we have a BoWVect which is a sparse vector for BoVW matching AND we have a FeatVect (a direct index)
        //  We will use the FeatVect for finding correspondences faster

        // Compute Bag of Words Vector
        mCurrentFrame.ComputeBoW();
//    cout << "Computed the BoW for this frame" << endl;

        // Relocalization is performed when tracking is lost
        // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation

        //1. Detect relocalization candidates for mCurrentFrame according to the BoVW model and all of them in a Keyframe* vector



        //1. Get the BoWVector for the current frame and iterate through all of its words
        //   The BoWVector is indexed by words and their associated weights
        //2. For each word, use the mvInvertedFile to find KF's that contain that specific word
        //3. For all those keyframes, put them in a vector as candidates, mark them as relocalization candidates for the current frame and count the number of shared words
        //4. Return if we have no keyframe relocalization candidates
        //5. Iterate through all keyframe relocalization candidates and find the maximum number of shared words between current frame and any relocalization keyframe
        //6. After that start computing their similarity scores ONLY for those that have greater similar words than 80%
        //  i. For each candidate, form an std::pair withe keyframe and BoVW SCORE while eliminating those kf's
        //     BoVW SCORE is calculated using the two BoWVec vectors which consist of visual words and their visibility scores
        //     So for each keyframe, we have reloc_query, reloc_words and reloc_score
        //7. Now iterate through the whole list of keyframes & their scores that share enough words
        //8. For each keyframe, query its covisibilty graph and get upto 10 keyframes neighbors for it
        //  i. Now using these neighbors, build an accumulative score for each keyframe by adding the scores of its neighbors
        //  ii. If a neighbor has more score than the keyframe, replace it and use the neighbor instead in the acc score keyframe pair
        //9. Using the 0.75 * bestScore, filter out the keyframes and return the rest as relocalization candidates
        vector<KeyFrame *> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);
        cout << "Relocalization candidates for this frame = " << vpCandidateKFs.size() << endl;

        const int NUMBER_OF_STARTING_KEYFRAMES = 10;

        // GPS Filter
        cout << "GPS filter disabled for now" << endl;
/*

        if (RobustFeatureSearch == true)
        {
            // Retrieve all those keyframes that are in close vicinity to the current position
//            float starting_position [3] = { mLastFrame.mTcw.at<float>(0,3), mLastFrame.mTcw.at<float>(1,3), mLastFrame.mTcw.at<float>(2,3)};
//            float starting_position [3] = {0};
//            if (mLastFrame == NULL)
//            {
//                for (int counter = 0; counter < 3; counter++)
//                    starting_position [counter] = 0.0;
//            }
//            else
//                for (int counter = 0; counter < 3; counter++)
//                    mLastFrame.mTcw.at<float>(counter,3);

            float starting_position [3] = { 0.0, 0.0, 0.0};
            const float radius = 10.0;
            vector<KeyFrame *> spatial_neighbors = mpMap->kfKDtree->GetNeighbors(starting_position, radius, true);

            // Printing the positions of all the neighbors
            cout << "Number of closeby neighbors are : " << spatial_neighbors.size() << endl;
            for (KeyFrame *kf: spatial_neighbors) {
                cout << kf->GetmnId() << ": " << kf->GetPoseInverse().at<float>(0, 3) << ", " << kf->GetPoseInverse().at<float>(1, 3)
                     << "," << kf->GetPoseInverse().at<float>(2, 3) << endl;
                vpCandidateKFs.push_back(kf);
            }
        }
	*/


        /*

//            cout << "REMOVE ME LATER" << endl;
//            for (int counter = 1; counter < NUMBER_OF_STARTING_KEYFRAMES;)
//            {
//                int internal_counter = 1;
//                KeyFrame *kf = allKfs.at(internal_counter);
//                while ( kf->GetmnId() != counter )
//                {
//                    kf = allKfs.at(++internal_counter);
//                }
////                if (kf->GetmnId() != counter)
////                {
////                    counter++;
////                    continue;
////                } else {
//                    vpCandidateKFs.push_back(kf);
//                    counter++;
////                }
//
//            }
//        }

*/

        if (vpCandidateKFs.empty()) {
            if (isDebugMode)
                cout << "No keyframe candidates from BoVW matching" << endl;
            return false;
        }
        else {
            if (isDebugMode)
                cout << "Number of candidates = " << vpCandidateKFs.size() << endl;
            for (KeyFrame* kf : vpCandidateKFs)
                if (isDebugMode)
                    cout << "Candidate = " << kf->mnId << endl;
        }

        const int nKFs = vpCandidateKFs.size();

        // We perform first an ORB matching with each candidate
        // If enough matches are found we setup a PnP solver
        ORBmatcher matcher(0.6, true);

        vector<PnPsolver *> vpPnPsolvers;
        vpPnPsolvers.resize(nKFs);

        vector<vector<MapPoint *> > vvpMapPointMatches;
        vvpMapPointMatches.resize(nKFs);

        vector<bool> vbDiscarded;
        vbDiscarded.resize(nKFs);

        int nCandidates = 0;

        for (int i = 0; i < nKFs; i++) {
            KeyFrame *pKF = vpCandidateKFs[i];
            if (pKF->isBad())
                vbDiscarded[i] = true;
            else {


                int nmatches = matcher.SearchByBoW(pKF, mCurrentFrame, vvpMapPointMatches[i]);

                if (isDebugMode)
                    cout << "Matching:: Number of matches by search by BoW = " << nmatches << endl;

                if (RobustFeatureSearch) {

                    int kdtreeMatches = matcher.SearchByKDTree(pKF, mCurrentFrame, mpMap, vvpMapPointMatches[i]);
                    if (isDebugMode)
                        cout << "Matching:: Number of matches by KD tree = " << kdtreeMatches << endl;

                    cout << "KF [" << vpCandidateKFs[i]->GetmnId() << "]" << ": BoW = " << nmatches << ", KD Tree = " << kdtreeMatches << endl;

                    nmatches = kdtreeMatches + nmatches;
                }

                int matches = 0;
                if (nmatches < 15) {
                    if (isDebugMode) {
//                        cout << "Discarding keyframe " << pKF->GetmnId() << " for only " << nmatches << " matches"
//                             << endl;
                        cout << "Further optimizing pose" << endl;

                        if (RobustFeatureSearch == true) {
                            if (PerformDiffOperation == true)
                            {
                                mCurrentFrame.SetPose(pKF->GetPose());
                                matches = matcher.SearchByKDTree(mCurrentFrame, mpMap);
                                cout << "Number of matches now = " << matches << endl;
                            }
                        }

                        for (int counter = 0; counter < mCurrentFrame.N; counter++)
                        {
                            if (mCurrentFrame.mvpMapPoints[counter] != NULL && vvpMapPointMatches[i][counter] == NULL)
                                vvpMapPointMatches[i][counter] = mCurrentFrame.mvpMapPoints[counter];
                        }
                    }

                    if (matches + nmatches < 15) {
                        cout << "Discarded for less than 15 matches" << endl;
                        vbDiscarded[i] = true;
                        continue;
                    }
                    else
                    {
                        if (isDebugMode)
                            cout << "KD-tree to the rescue : Keeping keyframe " << pKF->GetmnId() << " with  " << nmatches + matches  << " matches" << endl;
                        PnPsolver *pSolver = new PnPsolver(mCurrentFrame, vvpMapPointMatches[i]);
                        pSolver->SetRansacParameters(0.99, 8, 300, 4, 0.5, 5.991);
                        vpPnPsolvers[i] = pSolver;
                        nCandidates++;
                    }
                } else {
                    if (isDebugMode)
                        cout << "Keeping keyframe " << pKF->GetmnId() << " with  " << nmatches  << " matches" << endl;
                    PnPsolver *pSolver = new PnPsolver(mCurrentFrame, vvpMapPointMatches[i]);
                    pSolver->SetRansacParameters(0.99, 8, 300, 4, 0.5, 5.991);
                    vpPnPsolvers[i] = pSolver;
                    nCandidates++;
                }
            }
        }

        // Alternatively perform some iterations of P4P RANSAC
        // Until we found a camera pose supported by enough inliers
        bool bMatch = false;
        ORBmatcher matcher2(0.9, true);

        while (nCandidates > 0 && !bMatch) {
            for (int i = 0; i < nKFs; i++) {
                if (vbDiscarded[i])
                    continue;

                // Perform 5 Ransac Iterations
                vector<bool> vbInliers;
                int nInliers;
                bool bNoMore;

                PnPsolver *pSolver = vpPnPsolvers[i];
                cv::Mat Tcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers);

                // If Ransac reachs max. iterations discard keyframe
                if (bNoMore) {
                    if (isDebugMode)
                        cout << "RANSAC maxed out for keyframe " << vpCandidateKFs.at(i)->GetmnId() << endl;
                    vbDiscarded[i] = true;
                    nCandidates--;
                }

                // If a Camera Pose is computed, optimize
                if (!Tcw.empty()) {

                    if (isDebugMode)
                        cout << "We have obtained a pose for the current frame from the keyframe " << vpCandidateKFs.at(i)->GetmnId() << endl;

                    Tcw.copyTo(mCurrentFrame.mTcw);

                    set<MapPoint *> sFound;

                    const int np = vbInliers.size();

                    for (int j = 0; j < np; j++) {
                        if (vbInliers[j]) {
                            mCurrentFrame.mvpMapPoints[j] = vvpMapPointMatches[i][j];
                            sFound.insert(vvpMapPointMatches[i][j]);
                        } else
                            mCurrentFrame.mvpMapPoints[j] = NULL;
                    }
                    int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                    if (nGood < 10)
                    {
                        if (isDebugMode)
                            cout << "PoseOptimization could not retrieve more matches for keyframe " << vpCandidateKFs.at(i)->GetmnId() << endl;
                        continue;
                    }

                    for (int io = 0; io < mCurrentFrame.N; io++)
                        if (mCurrentFrame.mvbOutlier[io])
                            mCurrentFrame.mvpMapPoints[io] = static_cast<MapPoint *>(NULL);

                    if (nGood < 50) {
                        if (isDebugMode)
                            cout << "Performing a search by projection after finding " << nGood << " matches for keyframe " << vpCandidateKFs.at(i)->GetmnId() << endl;

                        int nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 10,
                                                                      100);

                        if (nadditional + nGood >= 50) {
                            nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                            // If many inliers but still not enough, search by projection again in a narrower window
                            // the camera has been already optimized with many points
                            if (nGood > 30 && nGood < 50) {
                                sFound.clear();
                                for (int ip = 0; ip < mCurrentFrame.N; ip++)
                                    if (mCurrentFrame.mvpMapPoints[ip])
                                        sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                                nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 3,
                                                                          64);

                                // Final optimization
                                if (nGood + nadditional >= 50) {
                                    nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                    for (int io = 0; io < mCurrentFrame.N; io++)
                                        if (mCurrentFrame.mvbOutlier[io])
                                            mCurrentFrame.mvpMapPoints[io] = NULL;
                                }
                            }
                        }
                        else
                        {
                            if (isDebugMode)
                                cout << "Total number of matches = " << nadditional + nGood << " < " << 50 << endl;
                        }
                    }


                    // If the pose is supported by enough inliers stop ransacs and continue
                    if (nGood >= 50) {
                        bMatch = true;
                        break;
                    }
//                    */
                    bMatch = true;

                }
            }
        }

        if (!bMatch) {
            if (isDebugMode)
                cout << "Failed stage 2" << endl;
            return false;
        } else {
            if (isDebugMode)
                cout << "Passed stage 2!" << endl;
            mnLastRelocFrameId = mCurrentFrame.mnId;
            return true;
        }

    }

    void Tracking::Reset() {

//    cout << "System Reseting" << endl;
        if (mpViewer) {
            mpViewer->RequestStop();
            while (!mpViewer->isStopped()) {
                std::this_thread::sleep_for(std::chrono::microseconds(3000));
            }
        }

        // Reset Local Mapping
//    cout << "Reseting Local Mapper...";
        mpLocalMapper->RequestReset();
//    cout << " done" << endl;

        // Reset Loop Closing
//    cout << "Reseting Loop Closing...";
        mpLoopClosing->RequestReset();
//    cout << " done" << endl;

        // Clear BoW Database
//    cout << "Reseting Database...";
        mpKeyFrameDB->clear();
//    cout << " done" << endl;

        // Clear Map (this erase MapPoints and KeyFrames)
        mpMap->clear();

        KeyFrame::nNextId = 0;
        Frame::nNextId = 0;
        mState = NO_IMAGES_YET;

        if (mpInitializer) {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer *>(NULL);
        }

        mlRelativeFramePoses.clear();
        mlpReferences.clear();
        mlFrameTimes.clear();
        mlbLost.clear();

        if (mpViewer)
            mpViewer->Release();
    }

    void Tracking::ChangeCalibration(const string &strSettingPath) {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        float fx = fSettings["Camera.fx"];
        float fy = fSettings["Camera.fy"];
        float cx = fSettings["Camera.cx"];
        float cy = fSettings["Camera.cy"];

        cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
        K.at<float>(0, 0) = fx;
        K.at<float>(1, 1) = fy;
        K.at<float>(0, 2) = cx;
        K.at<float>(1, 2) = cy;
        K.copyTo(mK);

        cv::Mat DistCoef(4, 1, CV_32F);
        DistCoef.at<float>(0) = fSettings["Camera.k1"];
        DistCoef.at<float>(1) = fSettings["Camera.k2"];
        DistCoef.at<float>(2) = fSettings["Camera.p1"];
        DistCoef.at<float>(3) = fSettings["Camera.p2"];
        const float k3 = fSettings["Camera.k3"];
        if (k3 != 0) {
            DistCoef.resize(5);
            DistCoef.at<float>(4) = k3;
        }
        DistCoef.copyTo(mDistCoef);

        mbf = fSettings["Camera.bf"];

        Frame::mbInitialComputations = true;
    }

    void Tracking::InformOnlyTracking(const bool &flag) {
        mbOnlyTracking = flag;
    }


} //namespace ORB_SLAM
