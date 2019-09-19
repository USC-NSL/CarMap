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

#include "KeyFrame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include<mutex>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
//#include <include/GlobalDefintions.h>

namespace ORB_SLAM2 {

    long unsigned int KeyFrame::nNextId = 0;

    KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB) :
            mnFrameId(F.mnId), mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
            mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
            mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
            mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
            fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
            mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn), mvLabels (F.mvLabels),
            mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
            mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
            mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
            mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
            mnMaxY(F.mnMaxY), mK(F.mK), mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),
            mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
            mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb / 2), mpMap(pMap), isLoadedFromDisk(false) {
        mnId = nNextId++;

        mGrid.resize(mnGridCols);
        for (int i = 0; i < mnGridCols; i++) {
            mGrid[i].resize(mnGridRows);
            for (int j = 0; j < mnGridRows; j++)
                mGrid[i][j] = F.mGrid[i][j];
        }

        SetPose(F.mTcw);
        fullyInitialized = true;
    }




    KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB, bool isExtraLabels) :
            mnFrameId(F.mnId), mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
            mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
            mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
            mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
            fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
            mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
            mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
            mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
            mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
            mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
            mnMaxY(F.mnMaxY), mK(F.mK), mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),
            mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
            mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb / 2), mpMap(pMap), isLoadedFromDisk(false),
            mvDPLabels(F.mvDPLabels), mvFDPLabels(F.mvFDPLabels), mvMNLabels(F.mvMNLabels), mvGTLabels (F.mvGTLabels){
        mnId = nNextId++;

        mGrid.resize(mnGridCols);
        for (int i = 0; i < mnGridCols; i++) {
            mGrid[i].resize(mnGridRows);
            for (int j = 0; j < mnGridRows; j++)
                mGrid[i][j] = F.mGrid[i][j];
        }

        mvLabels = F.mvLabels;
        mpsLabeled = true;
//
//        if (false) {
//            cout << "Skipping for " << mnId << endl;
//            mvLabels = F.mvLabels;
//            mpsLabeled = true;
//        }
//        else
//        {
//            cout << "All labels are 255" << endl;
//            mvLabels = std::vector <unsigned char> (N, 255);
//            mpsLabeled = false;
//        }

        SetPose(F.mTcw);
        fullyInitialized = true;
    }

    bool KeyFrame::AreMpsLabled ()
    {
        return mpsLabeled;
    }

//    void KeyFrame::InterpolateLabels ()
//    {
//        ORBmatcher matcher(0.6);
//
//    }
//



    KeyFrame::KeyFrame(KeyFrame *kf, Map *pMap, KeyFrameDatabase *pKFDB, cv::Mat transformationMatrix):

    mnFrameId(0), mTimeStamp(0.0),
    mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
    mnMaxX(FRAME_MN_MAX_X), mnMinX(FRAME_MN_MIN_Y), mnMaxY (FRAME_MN_MAX_Y), mnMinY (FRAME_MN_MIN_Y),
    mfGridElementWidthInv(0.0), mfGridElementHeightInv(0.0),
    mbFirstConnection (true), mpParent (static_cast<KeyFrame*> (NULL)),
    mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
    fx(0.0), fy(0.0), cx(0.0), cy(0.0), invfx(0.0), invfy(0.0),
    mbf(0.0), mb(0.0), mThDepth(0.0), N(0), mnScaleLevels(0), mfScaleFactor(0),
    mfLogScaleFactor(0.0),
    isLoadedFromDisk(true), mbNotErase (false), mbBad (false), mbToBeErased (false),
    mpMap(pMap)
    {
        bool isDebugMode = false;
        mnId = nNextId++;
        SetPose(kf->GetPose());

        if (isDebugMode)
            cout << "KF:  " << GetmnId() << ", adding " << kf->GetIndexedMapPoints().size() << " matched map points" << endl;
        std::vector <MapPoint*> mps = kf->GetIndexedMapPoints();
        for (MapPoint * mp : mps)
        {
            if (mp)
                if (!mp->isBad())
                    mvpMapPoints.push_back(mp);
        }

        if (isDebugMode)
            cout << "KF: " << GetmnId() << ", adding " << kf->GetOutliers().size() << " unmatched map points" << endl;
        std::vector <MapPoint*> outliers = kf->GetOutliers();
        int counter = 0;
        int alreadyAdded = 0;
        int newAdditions = 0;

        for (MapPoint * mp : outliers) {
            if (mp) {
                if (!mp->isBad()) {

                    cv::Mat pos = mp->GetWorldPos();
                    cv::Mat homogeneousPos = cv::Mat (1, 4, pos.type());
                    homogeneousPos.at<float>(0,0) = pos.at<float>(0,0);
                    homogeneousPos.at<float>(0,1) = pos.at<float>(0,1);
                    homogeneousPos.at<float>(0,2) = pos.at<float>(0,2);
                    homogeneousPos.at<float>(0,3) = 1;
                    homogeneousPos = transformationMatrix * homogeneousPos.t();
                    // Get all the meta-data for the map-point
                    cv::Mat descriptor = mp->GetDescriptor();
                    int numberOfObs = mp->nObs;
                    cv::Mat worldPos = mp->GetWorldPos();
                    unsigned char label = mp->GetSemanticLabel();
                    counter++;
                    MapPoint *mpInBaseMap;
                    if (mp->IsPresentInBaseMap() == true)// Is the map-point already added to the base map?
                    {
                        // Since the map-point is already present, we do not need to create it again
                        // We just retrieve the pointer to the map point
                        mpInBaseMap = mp->GetPtrInBaseMap();
                        alreadyAdded++;
                    }
                    else // The map-point has not been added to the base-map
                    {
                        // Create a new map-point
                        pos.at<float>(0,0) = homogeneousPos.at<float>(0,0);
                        pos.at<float>(0,1) = homogeneousPos.at<float>(0,1);
                        pos.at<float>(0,2) = homogeneousPos.at<float>(0,2);
                        mpInBaseMap = new MapPoint(pos, this, mpMap, numberOfObs, descriptor, label);
                        // Note that we added this map-point to the base map
                        mp->AddToBaseMap(mpInBaseMap);
                        newAdditions++;
                    }
                    mp->SetMapPointer(mpMap);
                    mp->ReconstructMpObservation();
                    mvpMapPoints.push_back(mpInBaseMap);
                    mpMap->AddMapPoint(mpInBaseMap);
                }
            }
        }

        if (isDebugMode) {
            cout << "Updated = " << alreadyAdded << " / " << outliers.size() << endl;
            cout << "New = " << newAdditions << " / " << outliers.size() << endl;
            cout << "Number of outliers belonging to this keyframe are " << counter << " / " << outliers.size() << endl;
        }

        fullyInitialized = true;
    }



    KeyFrame::KeyFrame(KeyFrame *kf, Map *pMap, KeyFrameDatabase *pKFDB, bool isOneShotRelocalization, cv::Mat transformationMatrix):

            mnFrameId(0), mTimeStamp(0.0),
            mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
            mnMaxX(FRAME_MN_MAX_X), mnMinX(FRAME_MN_MIN_Y), mnMaxY (FRAME_MN_MAX_Y), mnMinY (FRAME_MN_MIN_Y),
            mfGridElementWidthInv(0.0), mfGridElementHeightInv(0.0),
            mbFirstConnection (true), mpParent (static_cast<KeyFrame*> (NULL)),
            mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
            mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
            fx(0.0), fy(0.0), cx(0.0), cy(0.0), invfx(0.0), invfy(0.0),
            mbf(0.0), mb(0.0), mThDepth(0.0), N(0), mnScaleLevels(0), mfScaleFactor(0),
            mfLogScaleFactor(0.0),
            isLoadedFromDisk(true), mbNotErase (false), mbBad (false), mbToBeErased (false),
            mpMap(pMap)
    {
        bool isDebugMode = true;
        mnId = nNextId++;
        cv::Mat oldPose = kf->GetPoseInverse();
        cv::Mat transformedPose = transformationMatrix * oldPose;
        cv::Mat newPose = kf->GetPoseFromPoseInverse(transformedPose);
        SetPose(newPose);

        if (isDebugMode)
            cout << "KF: " << GetmnId() << ", adding " << kf->GetIndexedMapPoints().size() << " matched map points" << endl;
        std::vector <MapPoint*> mps = kf->GetIndexedMapPoints();
        for (MapPoint * mp : mps)
        {
            if (mp)
                if (!mp->isBad())
                    mvpMapPoints.push_back(mp);
        }

        if (isDebugMode)
            cout << "KF: " << GetmnId() << ", adding " << kf->GetOutliers().size() << " un-matched map points" << endl;
        std::vector <MapPoint*> outliers = kf->GetOutliers();
        int counter = 0;
        int alreadyAdded = 0;
        int newAdditions = 0;
        for (MapPoint * mp : outliers) {
            if (mp) {
                if (!mp->isBad()) {
                    // geometrical transformation for map point
                    cv::Mat pos = mp->GetWorldPos();
                    cv::Mat homogeneousPos = cv::Mat (1, 4, pos.type());
                    homogeneousPos.at<float>(0,0) = pos.at<float>(0,0);
                    homogeneousPos.at<float>(0,1) = pos.at<float>(0,1);
                    homogeneousPos.at<float>(0,2) = pos.at<float>(0,2);
                    homogeneousPos.at<float>(0,3) = 1;
                    homogeneousPos = transformationMatrix * homogeneousPos.t();
                    // retrieve meta-data for the  map-point
                    cv::Mat descriptor = mp->GetDescriptor();
                    int numberOfObs = mp->nObs;
                    cv::Mat worldPos = mp->GetWorldPos();
                    counter++;
                    // Add map-point to the keyframe and the map
                    // Is the map-point already added to the base map?
                    MapPoint *mpInBaseMap;
                    if (mp->IsPresentInBaseMap() == true)
                    {
                        // Since the map-point is already present, we do not need to create it again
                        // We just retrieve the pointer to the map point
                        mpInBaseMap = mp->GetPtrInBaseMap();
                        alreadyAdded++;
                    }
                    else // The map-point has not been added to the base-map
                    {
                        // Create a new map-point
                        pos.at<float>(0,0) = homogeneousPos.at<float>(0,0);
                        pos.at<float>(0,1) = homogeneousPos.at<float>(0,1);
                        pos.at<float>(0,2) = homogeneousPos.at<float>(0,2);
                        // Create a new map-point
                        mpInBaseMap = new MapPoint(pos, this, mpMap, numberOfObs, descriptor);
                        // Note that we added this map-point to the base map
                        mp->AddToBaseMap(mpInBaseMap);
                        newAdditions++;
                    }
                    mp->SetMapPointer(mpMap);
                    mp->ReconstructMpObservation();
                    mvpMapPoints.push_back(mpInBaseMap);
                    mpMap->AddMapPoint(mpInBaseMap);
                }
            }
        }

        fullyInitialized = true;
    }


    KeyFrame::KeyFrame (unsigned int id, cv::Mat m_Tcw, bool first_connection, KeyFrame* mp_parent, Map* diff_map)
    :
            mnFrameId(0), mTimeStamp(0.0),
            mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
            mnMaxX(FRAME_MN_MAX_X), mnMinX(FRAME_MN_MIN_Y), mnMaxY (FRAME_MN_MAX_Y), mnMinY (FRAME_MN_MIN_Y),
            mfGridElementWidthInv(0.0), mfGridElementHeightInv(0.0),
            mbFirstConnection (true), mpParent (static_cast<KeyFrame*> (NULL)),
            mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
            mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
            fx(0.0), fy(0.0), cx(0.0), cy(0.0), invfx(0.0), invfy(0.0),
            mbf(0.0), mb(0.0), mThDepth(0.0), N(0), mnScaleLevels(0), mfScaleFactor(0),
            mfLogScaleFactor(0.0),
            isLoadedFromDisk(true), mbNotErase (false), mbBad (false), mbToBeErased (false),
            mpMap(diff_map)
    {
        // copy the id
        mnId = id;
        Tcw = m_Tcw;
        mbFirstConnection = first_connection;
        mpParent = static_cast <KeyFrame*> (NULL);
        std::set <KeyFrame*> msp_childrens;
        mspChildrens = msp_childrens;
    }




    void KeyFrame::ReconstructKeyFrame (KeyFrameDatabase* ptrToKFDB, Map* ptrToMap)
    {
//        SetORBvocabulary (ptrToVocabFile); // Setup pointer to the vocabulary file
        SetKeyFrameDB (ptrToKFDB); // Setup pointer to the keyframe database
        ReconstructBoW (); // Reconstruct the BoVW vector
        ptrToKFDB->add(this); // add this keyframe to the keyframe database

//        cout << "Reconstructed KF in new map file" << endl;

    }

    void KeyFrame::ComputeBoW() {
        //Function: Compute BoW representation from key-point array
        //Method: Convert all descriptors to visual words and build their BoWVector + FeatureVector
        if (mBowVec.empty() || mFeatVec.empty()) {
            vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
            // Feature vector associate features with nodes in the 4th level (from leaves up)
            // We assume the vocabulary tree has 6 levels, change the 4 otherwise
            mpORBvocabulary->transform(vCurrentDesc, mBowVec, mFeatVec, Vocab_Level);
        }
        return;
    }

    void KeyFrame::ReconstructBoW() {
        //Function: Reconstruct BoW representation during map load using map points only
        //Method: Use map point array to construct BowVector and Feature Vector
        std::vector <MapPoint*> indexedMps = GetIndexedMapPoints();//Get all the indexed map points in this keyframe
        cv::Mat mpDescriptor;//Variable to store descriptors from all map points

//        cout << "Size = " << indexedMps.size() << endl;
        //populate mDescriptor with the descriptors from all map points
        for (MapPoint *mp : indexedMps)
            mpDescriptor.push_back( mp->GetDescriptor().clone() );

        //Clear the BoW vector and feature vector
        mBowVec.clear();
        mFeatVec.clear();
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mpDescriptor);//convert cv::Mat to vector<cv::Mat>

        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        mpORBvocabulary->transform(vCurrentDesc, mBowVec, mFeatVec, Vocab_Level);
    }

    cv::Mat KeyFrame::GetDescriptors() {
        //Return all key point descriptors
        return mDescriptors.clone();
    }

    void KeyFrame::SetDescriptors(cv::Mat descriptor) {
        cout << "Should set descriptors here" << endl;
//        mDescriptors = descriptor;
    }


    void KeyFrame::SetPose(const cv::Mat &Tcw_) {
        unique_lock<mutex> lock(mMutexPose);
        Tcw_.copyTo(Tcw);
        cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
        cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
        cv::Mat Rwc = Rcw.t();
        Ow = -Rwc * tcw;

        Twc = cv::Mat::eye(4, 4, Tcw.type());
        Rwc.copyTo(Twc.rowRange(0, 3).colRange(0, 3));
        Ow.copyTo(Twc.rowRange(0, 3).col(3));
        cv::Mat center = (cv::Mat_<float>(4, 1) << mHalfBaseline, 0, 0, 1);
        Cw = Twc * center;
    }

    cv::Mat KeyFrame::GetPoseInverseFromPose (const cv::Mat &Tcw_)
    {
        cv::Mat poseMat = cv::Mat::eye (4, 4, Tcw_.type());
        Tcw_.copyTo (poseMat);

        cv::Mat Rcw = poseMat.rowRange(0, 3).colRange(0, 3);
        cv::Mat tcw = poseMat.rowRange(0, 3).col(3);
        cv::Mat Rwc = Rcw.t();
        cv::Mat worldOrigin = -Rwc * tcw;

        cv::Mat inverseMat = cv::Mat::eye (4,4, Tcw_.type());
        Rwc.copyTo(inverseMat.rowRange(0, 3).colRange(0, 3));
        worldOrigin.copyTo(inverseMat.rowRange(0, 3).col(3));

        return inverseMat;
    }

    cv::Mat KeyFrame::GetPoseFromPoseInverse (const cv::Mat &Twc_)
    {
        cv::Mat inversePoseMat = cv::Mat::eye (4, 4, Twc_.type());
        Twc_.copyTo (inversePoseMat);

        cv::Mat Rwc = inversePoseMat.rowRange(0, 3).colRange(0, 3);
        cv::Mat twc = inversePoseMat.rowRange(0, 3).col(3);
        cv::Mat Rcw = Rwc.t();
        cv::Mat worldOrigin = -Rcw * twc;

        cv::Mat poseMat = cv::Mat::eye (4,4, Twc_.type());
        Rcw.copyTo(poseMat.rowRange(0, 3).colRange(0, 3));
        worldOrigin.copyTo(poseMat.rowRange(0, 3).col(3));

        return poseMat;


    }


    cv::Mat KeyFrame::GetPose() {
        unique_lock<mutex> lock(mMutexPose);
        return Tcw.clone();
    }

    cv::Mat KeyFrame::GetPoseInverse() {
        unique_lock<mutex> lock(mMutexPose);
        return Twc.clone();
    }

    cv::Mat KeyFrame::GetCameraCenter() {
        unique_lock<mutex> lock(mMutexPose);
        return Ow.clone();
    }

    cv::Mat KeyFrame::GetStereoCenter() {
        unique_lock<mutex> lock(mMutexPose);
        return Cw.clone();
    }


    cv::Mat KeyFrame::GetRotation() {
        unique_lock<mutex> lock(mMutexPose);
        return Tcw.rowRange(0, 3).colRange(0, 3).clone();
    }

    cv::Mat KeyFrame::GetTranslation() {
        unique_lock<mutex> lock(mMutexPose);
        return Tcw.rowRange(0, 3).col(3).clone();
    }

    void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight) {

//        cout << "AddConnection: Acquiring lock for " << this->mnId << endl;

        {
            unique_lock<mutex> lock(mMutexConnections);
            if (!mConnectedKeyFrameWeights.count(pKF))
                mConnectedKeyFrameWeights[pKF] = weight;
            else if (mConnectedKeyFrameWeights[pKF] != weight)
                mConnectedKeyFrameWeights[pKF] = weight;
            else
                return;
        }

//        cout << "AddConnection(): Release lock for " << this->mnId << endl;




        std::chrono::steady_clock::time_point updateStart = std::chrono::steady_clock::now();

        UpdateBestCovisibles();
        std::chrono::steady_clock::time_point updateEnd = std::chrono::steady_clock::now();
        updateBestCovisibles += std::chrono::duration_cast<std::chrono::microseconds>(updateEnd-updateStart).count();



    }

    void KeyFrame::UpdateBestCovisibles() {
//        cout << "UpdateBestCovisibles(): Acquiring lock for " << this->mnId << endl;

        unique_lock<mutex> lock(mMutexConnections);
        vector<pair<int, KeyFrame *> > vPairs;
        vPairs.reserve(mConnectedKeyFrameWeights.size());
        for (map<KeyFrame *, int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend = mConnectedKeyFrameWeights.end();
             mit != mend; mit++)
            vPairs.push_back(make_pair(mit->second, mit->first));

        sort(vPairs.begin(), vPairs.end());
        list<KeyFrame *> lKFs;
        list<int> lWs;
        for (size_t i = 0, iend = vPairs.size(); i < iend; i++) {
            lKFs.push_front(vPairs[i].second);
            lWs.push_front(vPairs[i].first);
        }

        mvpOrderedConnectedKeyFrames = vector<KeyFrame *>(lKFs.begin(), lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
//        cout << "UpdateBestCovisibles(): Release lock for " << this->mnId << endl;


    }

    set<KeyFrame *> KeyFrame::GetConnectedKeyFrames() {
//        cout << "GetConnectedKeyFrames(): Acquiring lock for " << this->mnId << endl;

        unique_lock<mutex> lock(mMutexConnections);
        set<KeyFrame *> s;
        for (map<KeyFrame *, int>::iterator mit = mConnectedKeyFrameWeights.begin();
             mit != mConnectedKeyFrameWeights.end(); mit++)
            s.insert(mit->first);

//        cout << "GetConnectedKeyFrames(): Release lock for " << this->mnId << endl;

        return s;
    }

    vector<KeyFrame *> KeyFrame::GetVectorCovisibleKeyFrames() {
        unique_lock<mutex> lock(mMutexConnections);
        return mvpOrderedConnectedKeyFrames;
    }

    vector<KeyFrame *> KeyFrame::GetBestCovisibilityKeyFrames(const int &N) {
        unique_lock<mutex> lock(mMutexConnections);
        if ((int) mvpOrderedConnectedKeyFrames.size() < N)
            return mvpOrderedConnectedKeyFrames;
        else
            return vector<KeyFrame *>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin() + N);

    }

    vector<KeyFrame *> KeyFrame::GetCovisiblesByWeight(const int &w) {
        unique_lock<mutex> lock(mMutexConnections);

        if (mvpOrderedConnectedKeyFrames.empty())
            return vector<KeyFrame *>();

        vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(), mvOrderedWeights.end(), w,
                                               KeyFrame::weightComp);
        if (it == mvOrderedWeights.end())
            return vector<KeyFrame *>();
        else {
            int n = it - mvOrderedWeights.begin();
            return vector<KeyFrame *>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin() + n);
        }
    }

    int KeyFrame::GetWeight(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexConnections);
        if (mConnectedKeyFrameWeights.count(pKF))
            return mConnectedKeyFrameWeights[pKF];
        else
            return 0;
    }

    void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx) {
        unique_lock<mutex> lock(mMutexFeatures);
        mvpMapPoints[idx] = pMP;
    }

    void KeyFrame::EraseMapPointMatch(const size_t &idx) {
        unique_lock<mutex> lock(mMutexFeatures);
        mvpMapPoints[idx] = static_cast<MapPoint *>(NULL);
    }

    void KeyFrame::EraseMapPointMatch(MapPoint *pMP) {
        int idx = pMP->GetIndexInKeyFrame(this);
        if (idx >= 0)
            mvpMapPoints[idx] = static_cast<MapPoint *>(NULL);
    }


    void KeyFrame::ReplaceMapPointMatch(const size_t &idx, MapPoint *pMP) {
        mvpMapPoints[idx] = pMP;
    }

    long unsigned int KeyFrame::GetmnId()
    {
        return this->mnId;
    }

    bool KeyFrame::GetIsFirstConnection() {
        return mbFirstConnection;

    }

    void KeyFrame::PrintLabelStatistics( ) {

        cout << "KF [ " << GetmnId() << " ] ----- Label statistics " << endl;
        set <MapPoint *> allMps = GetMapPoints();

        int labelCounters [ SEMANTIC_SEGMENTOR_NUMBER_OF_OBJECTS ] = {0};

        for (MapPoint * mp : allMps)
        {
            labelCounters [mp->GetSemanticLabel()]++;
        }

        for ( unsigned char counter = 0; counter < SEMANTIC_SEGMENTOR_NUMBER_OF_OBJECTS; counter++ )
        {
            cout << SemanticSegmentor::GetObjectName(counter) << " = " << ( ( labelCounters [counter] * 100.0 ) / allMps.size() )<< endl;
        }

    }


    set<MapPoint *> KeyFrame::GetMapPoints() {
        unique_lock<mutex> lock(mMutexFeatures);
        set<MapPoint *> s;
        for (size_t i = 0, iend = mvpMapPoints.size(); i < iend; i++) {
            if (!mvpMapPoints[i])
                continue;
            MapPoint *pMP = mvpMapPoints[i];
            if (!pMP->isBad())
                s.insert(pMP);
        }
        return s;
    }



    int KeyFrame::TrackedMapPoints(const int &minObs) {
        unique_lock<mutex> lock(mMutexFeatures);

        int nPoints = 0;
        const bool bCheckObs = minObs > 0;
        for (int i = 0; i < N; i++) {
            MapPoint *pMP = mvpMapPoints[i];
            if (pMP) {
                if (!pMP->isBad()) {
                    if (bCheckObs) {
                        if (mvpMapPoints[i]->Observations() >= minObs)
                            nPoints++;
                    } else
                        nPoints++;
                }
            }
        }

        return nPoints;
    }

    vector<MapPoint *> KeyFrame::GetMapPointMatches() {
        unique_lock<mutex> lock(mMutexFeatures);
        return mvpMapPoints;
    }

    void KeyFrame::ClearMapPointMatches ()
    {
        {
            unique_lock<mutex> lock(mMutexFeatures);
            for (size_t mpCounter = 0; mpCounter < mvpMapPoints.size(); mpCounter++) {
                mvpMapPoints[mpCounter] = static_cast <MapPoint *> (NULL);
            }

        }
        SyncMapPoints();
        return;
    }

    void KeyFrame::CreateDuplicateMapPoints ()
    {
        bool isDebugMode = false;

        if (isDebugMode)
        cout << "Creating duplicate map-points" << endl;
        {
            unique_lock<mutex> lock(mMutexFeatures);
            for (size_t mpCounter = 0; mpCounter < mvpMapPoints.size(); mpCounter++) {

                MapPoint *pMp = mvpMapPoints[mpCounter];
                if (pMp)
                    if (!pMp->isBad())
                        dupMvpMPs.push_back(pMp);
            }
        }

        if (isDebugMode) {
            cout << "Size of mvpMapPoints before doing anything = " << GetIndexedMapPoints().size() << endl;
            cout << "Size of outliers before removing inliers = " << GetOutliers().size() << endl;
        }
        return;
    }

    vector<MapPoint*> KeyFrame::GetOutliers ()
    {

        const bool isDebugMode = false;
        vector<MapPoint*> validDups;
//        unique_lock<mutex> lock(mMutexFeatures);
//        cout << "Returning outliers with dupMvpSize = " << dupMvpMPs.size() << endl;
        for (size_t mpCounter = 0; mpCounter < dupMvpMPs.size(); mpCounter++) {
            MapPoint* pMp = dupMvpMPs[mpCounter];
            if (pMp != NULL) {
                if (!pMp->isBad())
                    validDups.push_back(pMp);
            }
        }

        if (isDebugMode) {
            cout << "Unmatched map points = " << validDups.size() << endl;
            cout << "Matched map points = " << GetIndexedMapPoints().size() << endl;
        }
        return validDups;
    }

    MapPoint *KeyFrame::GetMapPoint(const size_t &idx) {
        unique_lock<mutex> lock(mMutexFeatures);
        return mvpMapPoints[idx];
    }

    void KeyFrame::UpdateConnections(bool isReconstruction) {

        map<KeyFrame *, int> KFcounter;
        vector<MapPoint *> vpMP;

//        if (isReconstruction == false)
        {
            unique_lock<mutex> lockMPs(mMutexFeatures);
            vpMP = mvpMapPoints;
        }
//        else
//            vpMP = mvpMapPoints;

        // Find number of connections with each keyframe
        for (vector<MapPoint *>::iterator vit = vpMP.begin(), vend = vpMP.end(); vit != vend; vit++) {
            MapPoint *pMP = *vit;

            if (!pMP)
                continue;

            if (pMP->isBad())
                continue;

            map<KeyFrame *, MapPoint_Observation> observations = pMP->GetMpObservations();

            for (map<KeyFrame *, MapPoint_Observation>::iterator mit = observations.begin(), mend = observations.end();
                 mit != mend; mit++) {
                if (mit->first->mnId == mnId)
                    continue;
                KFcounter[mit->first]++;
            }

        }

        if (KFcounter.empty())
        {
//            cout << "Empty" << endl;
            return;
        }

        //If the counter is greater than threshold add connection
        //In case no keyframe counter is over threshold add the one with maximum counter
        int nmax = 0;
        KeyFrame *pKFmax = NULL;
        int th = 15;


        // Filter out all those with less than threshold connections
        vector<pair<int, KeyFrame *> > vPairs;
        vPairs.reserve(KFcounter.size());
        for (map<KeyFrame *, int>::iterator mit = KFcounter.begin(), mend = KFcounter.end(); mit != mend; mit++) {
            if (mit->second > nmax) {
                nmax = mit->second;
                pKFmax = mit->first;
            }


            if (mit->second >= th) {
                vPairs.push_back(make_pair(mit->second, mit->first));

                if (isReconstruction == false)
                    (mit->first)->AddConnection(this, mit->second);

            }



        }

        if (vPairs.empty()) {
            vPairs.push_back(make_pair(nmax, pKFmax));
            pKFmax->AddConnection(this, nmax);
        }



        // Sort the keyframes based on the number of connections

        sort(vPairs.begin(), vPairs.end());

        list<KeyFrame *> lKFs;
        list<int> lWs;
        for (size_t i = 0; i < vPairs.size(); i++) {
            lKFs.push_front(vPairs[i].second);
            lWs.push_front(vPairs[i].first);
        }
//        if (isReconstruction == false)
        {
            unique_lock<mutex> lockCon(mMutexConnections);

            // mspConnectedKeyFrames = spConnectedKeyFrames;
            mConnectedKeyFrameWeights = KFcounter;
            mvpOrderedConnectedKeyFrames = vector<KeyFrame *>(lKFs.begin(), lKFs.end());
            mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

            if (mbFirstConnection && mnId != 0) {
                mpParent = mvpOrderedConnectedKeyFrames.front();
                mpParent->AddChild(this);
//                cout << "Parent of " << this->mnId << " is " << GetParent()->mnId << endl;
                mbFirstConnection = false;
//                cout << "First connection of KF [ " << this->GetmnId() << " ] = " << mbFirstConnection << endl;
            }

        }
//        else
//        {
//            mConnectedKeyFrameWeights = KFcounter;
//            mvpOrderedConnectedKeyFrames = vector<KeyFrame *>(lKFs.begin(), lKFs.end());
//            mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
//
//            if (mbFirstConnection && mnId != 0) {
//                mpParent = mvpOrderedConnectedKeyFrames.front();
//                mpParent->AddChild(this);
////                cout << "Parent of " << this->mnId << " is " << GetParent()->mnId << endl;
//                mbFirstConnection = false;
////                cout << "First connection of KF [ " << this->GetmnId() << " ] = " << mbFirstConnection << endl;
//            }
//
//        }


    }


    float KeyFrame::GetAverageConnectedKeyFrameWeight() {

        unsigned long int totalWeights = 0;
        for (int weight : mvOrderedWeights)
            totalWeights += weight;

        return (totalWeights / (mvOrderedWeights.size() * 1.0));
    }



    //TODO comment this
    void KeyFrame::OldUpdateConnections(){
        map<KeyFrame *, int> KFcounter;

        vector<MapPoint *> vpMP;

        {
            unique_lock<mutex> lockMPs(mMutexFeatures);
            vpMP = mvpMapPoints;
        }

        //For all map points in keyframe check in which other keyframes are they seen
        //Increase counter for those keyframes
        for (vector<MapPoint *>::iterator vit = vpMP.begin(), vend = vpMP.end(); vit != vend; vit++) {
            MapPoint *pMP = *vit;

            if (!pMP)
                continue;

            if (pMP->isBad())
                continue;

//            map<KeyFrame *, size_t> observations = pMP->GetObservations();
            map<KeyFrame *, size_t > observations = pMP->GetObservations();

            for (map<KeyFrame *, size_t >::iterator mit = observations.begin(), mend = observations.end();
                 mit != mend; mit++) {
                if (mit->first->mnId == mnId)
                    continue;
                KFcounter[mit->first]++;
            }
        }

        // This should not happen
        if (KFcounter.empty())
            return;

        //If the counter is greater than threshold add connection
        //In case no keyframe counter is over threshold add the one with maximum counter
        int nmax = 0;
        KeyFrame *pKFmax = NULL;
        int th = 15;

        vector<pair<int, KeyFrame *> > vPairs;
        vPairs.reserve(KFcounter.size());
        for (map<KeyFrame *, int>::iterator mit = KFcounter.begin(), mend = KFcounter.end(); mit != mend; mit++) {
            if (mit->second > nmax) {
                nmax = mit->second;
                pKFmax = mit->first;
            }
            if (mit->second >= th) {
                vPairs.push_back(make_pair(mit->second, mit->first));
                (mit->first)->AddConnection(this, mit->second);
            }
        }

        if (vPairs.empty()) {
            vPairs.push_back(make_pair(nmax, pKFmax));
            pKFmax->AddConnection(this, nmax);
        }

        sort(vPairs.begin(), vPairs.end());
        list<KeyFrame *> lKFs;
        list<int> lWs;
        for (size_t i = 0; i < vPairs.size(); i++) {
            lKFs.push_front(vPairs[i].second);
            lWs.push_front(vPairs[i].first);
        }

        {
            unique_lock<mutex> lockCon(mMutexConnections);

            // mspConnectedKeyFrames = spConnectedKeyFrames;
            mConnectedKeyFrameWeights = KFcounter;
            mvpOrderedConnectedKeyFrames = vector<KeyFrame *>(lKFs.begin(), lKFs.end());
            mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

            if (mbFirstConnection && mnId != 0) {
                mpParent = mvpOrderedConnectedKeyFrames.front();
                mpParent->AddChild(this);
                mbFirstConnection = false;
            }

        }
    }

    bool KeyFrame::GetFirstConnection ()
    {
        return mbFirstConnection;
    }

    void KeyFrame::AddChild(KeyFrame *pKF) {
//        cout << "Added " << pKF->mnId << endl;
        unique_lock<mutex> lockCon(mMutexConnections);
        mspChildrens.insert(pKF);
    }

    void KeyFrame::EraseChild(KeyFrame *pKF) {
        unique_lock<mutex> lockCon(mMutexConnections);
        mspChildrens.erase(pKF);
    }

    void KeyFrame::ChangeParent(KeyFrame *pKF) {
        unique_lock<mutex> lockCon(mMutexConnections);
        mpParent = pKF;
        pKF->AddChild(this);
    }

    set<KeyFrame *> KeyFrame::GetChilds() {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mspChildrens;
    }

    KeyFrame *KeyFrame::GetParent() {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mpParent;
    }

    bool KeyFrame::hasChild(KeyFrame *pKF) {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mspChildrens.count(pKF);
    }

    void KeyFrame::AddLoopEdge(KeyFrame *pKF) {
        unique_lock<mutex> lockCon(mMutexConnections);
        mbNotErase = true;
        mspLoopEdges.insert(pKF);
    }

    set<KeyFrame *> KeyFrame::GetLoopEdges() {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mspLoopEdges;
    }

    void KeyFrame::SetNotErase() {
        unique_lock<mutex> lock(mMutexConnections);
        mbNotErase = true;
    }

    void KeyFrame::SetErase() {
        {
            unique_lock<mutex> lock(mMutexConnections);
            if (mspLoopEdges.empty()) {
                mbNotErase = false;
            }
        }

        if (mbToBeErased) {
            SetBadFlag();
        }
    }

    void KeyFrame::SetBadFlag() {
        {
            unique_lock<mutex> lock(mMutexConnections);
            if (mnId == 0)
                return;
            else if (mbNotErase) {
                mbToBeErased = true;
                return;
            }
        }

        for (map<KeyFrame *, int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend = mConnectedKeyFrameWeights.end();
             mit != mend; mit++)
            mit->first->EraseConnection(this);

        for (size_t i = 0; i < mvpMapPoints.size(); i++)
            if (mvpMapPoints[i])
                mvpMapPoints[i]->EraseObservation(this);
        {
            unique_lock<mutex> lock(mMutexConnections);
            unique_lock<mutex> lock1(mMutexFeatures);

            mConnectedKeyFrameWeights.clear();
            mvpOrderedConnectedKeyFrames.clear();

            // Update Spanning Tree
            set<KeyFrame *> sParentCandidates;
            sParentCandidates.insert(mpParent);

            // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
            // Include that children as new parent candidate for the rest
            while (!mspChildrens.empty()) {
                bool bContinue = false;

                int max = -1;
                KeyFrame *pC;
                KeyFrame *pP;

                for (set<KeyFrame *>::iterator sit = mspChildrens.begin(), send = mspChildrens.end();
                     sit != send; sit++) {
                    KeyFrame *pKF = *sit;
                    if (pKF->isBad())
                        continue;

                    // Check if a parent candidate is connected to the keyframe
                    vector<KeyFrame *> vpConnected = pKF->GetVectorCovisibleKeyFrames();
                    for (size_t i = 0, iend = vpConnected.size(); i < iend; i++) {
                        for (set<KeyFrame *>::iterator spcit = sParentCandidates.begin(), spcend = sParentCandidates.end();
                             spcit != spcend; spcit++) {
                            if (vpConnected[i]->mnId == (*spcit)->mnId) {
                                int w = pKF->GetWeight(vpConnected[i]);
                                if (w > max) {
                                    pC = pKF;
                                    pP = vpConnected[i];
                                    max = w;
                                    bContinue = true;
                                }
                            }
                        }
                    }
                }

                if (bContinue) {
                    pC->ChangeParent(pP);
                    sParentCandidates.insert(pC);
                    mspChildrens.erase(pC);
                } else
                    break;
            }

            // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
            if (!mspChildrens.empty())
                for (set<KeyFrame *>::iterator sit = mspChildrens.begin(); sit != mspChildrens.end(); sit++) {
                    (*sit)->ChangeParent(mpParent);
                }

            mpParent->EraseChild(this);
            mTcp = Tcw * mpParent->GetPoseInverse();
            mbBad = true;
        }


        mpMap->EraseKeyFrame(this);
        mpKeyFrameDB->erase(this);
    }

    bool KeyFrame::isBad() {
//        cout << "isBad(): Acquiring lock for " << this->mnId << std::flush << endl;
//        if (GetmnId() == 59 || GetmnId() == 60)
//            return true;
        try {
            unique_lock<mutex> lock(mMutexConnections);
        }
        catch (std::exception &e)
        {
            cerr << "Exception caught : " << e.what() << std::flush << endl;
        }
        return mbBad;
    }

    bool KeyFrame::isBadReconstruction ()
    {
//        cout << "isBad(): Acquiring lock for " << this->mnId << std::flush;
//        cout << std::flush << endl;

//        if (this->mnId == 190)
//            return true;
//        if (fullyInitialized == true))
            return mbBad;
//        else
//        {
//            cerr << "Skipping " << GetmnId() << endl;
//            return true;
//
//        }
    }

    void KeyFrame::EraseConnection(KeyFrame *pKF) {
        bool bUpdate = false;
        {

//            cout << "EraseConnection(): Acquiring lock for " << this->mnId << endl;
            unique_lock<mutex> lock(mMutexConnections);
            if (mConnectedKeyFrameWeights.count(pKF)) {
                mConnectedKeyFrameWeights.erase(pKF);
                bUpdate = true;
            }
        }

//        cout << "EraseConnection(): Releasing lock for " << this->mnId << endl;


        if (bUpdate)
            UpdateBestCovisibles();
    }

    vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const {
        vector<size_t> vIndices;
        vIndices.reserve(N);

        const int nMinCellX = max(0, (int) floor((x - mnMinX - r) * mfGridElementWidthInv));
        if (nMinCellX >= mnGridCols)
            return vIndices;

        const int nMaxCellX = min((int) mnGridCols - 1, (int) ceil((x - mnMinX + r) * mfGridElementWidthInv));
        if (nMaxCellX < 0)
            return vIndices;

        const int nMinCellY = max(0, (int) floor((y - mnMinY - r) * mfGridElementHeightInv));
        if (nMinCellY >= mnGridRows)
            return vIndices;

        const int nMaxCellY = min((int) mnGridRows - 1, (int) ceil((y - mnMinY + r) * mfGridElementHeightInv));
        if (nMaxCellY < 0)
            return vIndices;

        for (int ix = nMinCellX; ix <= nMaxCellX; ix++) {
            for (int iy = nMinCellY; iy <= nMaxCellY; iy++) {
                const vector<size_t> vCell = mGrid[ix][iy];
                for (size_t j = 0, jend = vCell.size(); j < jend; j++) {
                    const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                    const float distx = kpUn.pt.x - x;
                    const float disty = kpUn.pt.y - y;

                    if (fabs(distx) < r && fabs(disty) < r)
                        vIndices.push_back(vCell[j]);
                }
            }
        }

        return vIndices;
    }

    bool KeyFrame::IsInImage(const float &x, const float &y) const {
        return (x >= mnMinX && x < mnMaxX && y >= mnMinY && y < mnMaxY);
    }

    cv::Mat KeyFrame::UnprojectStereo(int i) {
        const float z = mvDepth[i];
        if (z > 0) {
            const float u = mvKeys[i].pt.x;
            const float v = mvKeys[i].pt.y;
            const float x = (u - cx) * z * invfx;
            const float y = (v - cy) * z * invfy;
            cv::Mat x3Dc = (cv::Mat_<float>(3, 1) << x, y, z);

            unique_lock<mutex> lock(mMutexPose);
            return Twc.rowRange(0, 3).colRange(0, 3) * x3Dc + Twc.rowRange(0, 3).col(3);
        } else
            return cv::Mat();
    }

    cv::Mat KeyFrame::UnprojectStereo (float xx, float yy, float z)
    {
        if (z > 0) {
            const float u = xx;
            const float v = yy;
            const float x = (u - cx) * z * invfx;
            const float y = (v - cy) * z * invfy;
            cv::Mat x3Dc = (cv::Mat_<float>(3, 1) << x, y, z);

            unique_lock<mutex> lock(mMutexPose);
            return Twc.rowRange(0, 3).colRange(0, 3) * x3Dc + Twc.rowRange(0, 3).col(3);
        } else
            return cv::Mat();
    }

    std::vector <MapPoint *> KeyFrame::GetIndexedMapPoints ()
    {
        SyncMapPoints();//Update indexedMapPoint array
        return indexedMapPoints;
    }

    void KeyFrame::SyncMapPoints ()
    {
        //Function: update indexedMapPoints
        //Method: get all map points for the current keyframe and store them in a vector
        std::set <MapPoint*> allMps = GetMapPoints();//Get set of map points
        indexedMapPoints.clear();//clear the already present map points
        for (MapPoint * mp : allMps)
            indexedMapPoints.push_back(mp);
        return;
    }

    void KeyFrame::PrintAllMapPoints2DCoordinates ()
    {
        std::vector <ORB_SLAM2::MapPoint*> allMps = this->GetIndexedMapPoints();

        for (ORB_SLAM2::MapPoint *mp : allMps)
        {
            std::map <KeyFrame*, MapPoint_Observation> mpObsMap = mp->GetMpObservations();
            for (std::pair <KeyFrame*, MapPoint_Observation> mpObs : mpObsMap)
            {
                cout << "pMP, KF [ " << mp->mnId << ", " << mpObs.first->GetmnId() << " ] --- ( x , y ) = "
                     << mpObs.second.Keypoint.pt.x << ", " << mpObs.second.Keypoint.pt.y << " )" << endl;
            }

            cout << endl;
        }
    }

    void KeyFrame::PrintMapPointIndices ()
    {
        std::vector <MapPoint*> mpMatches = indexedMapPoints;

        int counter = 0;
        int mpIndex = 0;
        for (auto mp : mpMatches)
        {
            if (mp != NULL)
                cout << mpIndex++ << " mp [" << mp->mnId << "] = " << counter << endl;
            counter++;
        }
    }


    void KeyFrame::calculateUndistoredKeyPoints(void) {
        cv::FileStorage fSettings(pathToCameraCalibration, cv::FileStorage::READ);
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
        }\

        float fps = fSettings["Camera.fps"];
        if (fps == 0)
            fps = 30;

        // Max/Min Frames to insert keyframes and to check relocalisation

        if (DistCoef.at<float>(0) == 0.0) {
            mvKeysUn = mvKeys;
            return;
        }

        // Fill matrix with points
        cv::Mat mat(N, 2, CV_32F);
        for (int i = 0; i < N; i++) {
            mat.at<float>(i, 0) = mvKeys[i].pt.x;
            mat.at<float>(i, 1) = mvKeys[i].pt.y;
        }

        // Undistort points
        mat = mat.reshape(2);
        cv::undistortPoints(mat, mat, mK, DistCoef, cv::Mat(), mK);
        mat = mat.reshape(1);

        // Fill undistorted keypoint vector
        mvKeysUn.resize(N);
        for (int i = 0; i < N; i++) {
            cv::KeyPoint kp = mvKeys[i];
            kp.pt.x = mat.at<float>(i, 0);
            kp.pt.y = mat.at<float>(i, 1);
            mvKeysUn[i] = kp;
        }


    }


    float KeyFrame::ComputeSceneMedianDepth(const int q) {
        vector<MapPoint *> vpMapPoints;
        cv::Mat Tcw_;
        {
            unique_lock<mutex> lock(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPose);
            vpMapPoints = mvpMapPoints;
            Tcw_ = Tcw.clone();
        }

        vector<float> vDepths;
        vDepths.reserve(N);
        cv::Mat Rcw2 = Tcw_.row(2).colRange(0, 3);
        Rcw2 = Rcw2.t();
        float zcw = Tcw_.at<float>(2, 3);
        for (int i = 0; i < N; i++) {
            if (mvpMapPoints[i]) {
                MapPoint *pMP = mvpMapPoints[i];
                cv::Mat x3Dw = pMP->GetWorldPos();
                float z = Rcw2.dot(x3Dw) + zcw;
                vDepths.push_back(z);
            }
        }

        sort(vDepths.begin(), vDepths.end());

        return vDepths[(vDepths.size() - 1) / q];
    }


    void KeyFrame::SetKeyFrameDB(ORB_SLAM2::KeyFrameDatabase *kfDbPtr) {
        mpKeyFrameDB = kfDbPtr;
    }

    void KeyFrame::SetMapPointer(ORB_SLAM2::Map *ptrToMap) {
        mpMap = ptrToMap;
    }

    void KeyFrame::ReconstructCoVisibilityGraph ()
    {
        //Clear vectors

        {
//            cout << "ReconstructCoVisibilityGraph(): Acquiring lock for " << this->mnId << endl;
//            unique_lock<mutex> lockCon(mMutexConnections);
            mConnectedKeyFrameWeights.clear();
            mvpOrderedConnectedKeyFrames.clear();
            mvOrderedWeights.clear();
        }

//        cout << "ReconstructionCoVisibilityGraph(): Releasing lock for " << this->mnId << endl;
        //Update co-visibilty connections
        UpdateConnections(true);
    }

    void KeyFrame::OldReconstructCoVisibilityGraph ()
    {
        //Clear vectors

        {

            unique_lock<mutex> lockCon(mMutexConnections);
            mConnectedKeyFrameWeights.clear();
            mvpOrderedConnectedKeyFrames.clear();
            mvOrderedWeights.clear();
        }

        //Update co-visibilty connections
        OldUpdateConnections();
    }

    void KeyFrame::SetCameraCalibration(float *CamCalib) {
        fx = CamCalib[0];
        fy = CamCalib[1];
        cx = CamCalib[2];
        cy = CamCalib[3];
        mbf = CamCalib[4];
        mThDepth = CamCalib[5];

        //Reconstructable
        invfx = 1.0f / fx;
        invfy = 1.0f / fy;
        mb = mbf / fx;
        mHalfBaseline = mb / 2 ;


        cv::Mat K = cv::Mat::eye(3,3,CV_32F);
        K.at<float>(0,0) = fx;
        K.at<float>(1,1) = fy;
        K.at<float>(0,2) = cx;
        K.at<float>(1,2) = cy;

        K.copyTo(mK);




    }
    void KeyFrame::StoreMapPointObservation ()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        for (size_t mpCounter = 0; mpCounter < mvpMapPoints.size(); mpCounter++)
        {
            if ( !mvpMapPoints [mpCounter] )
                continue;
            else
            {
                if (mvpMapPoints[mpCounter]->isBad())
                    continue;
                else
                {
                    MapPoint * mp = mvpMapPoints [mpCounter];
                    mp->detectionKeyFrames.insert( std::pair <KeyFrame *, size_t > (this, mpCounter) );
                }
            }

        }
    }

    bool KeyFrame::LoadedFromDisk() {
        return isLoadedFromDisk;
    }

    void KeyFrame::AssignScaleVariables (float scaleLevel, float scaleFactor, float logScaleFactor,
    std::vector <float> scaleFactorsVec, std::vector <float> invScaleFactorsVec, std::vector <float> levelSigma, std::vector <float> invLevelSigma)
    {

        mnScaleLevels = scaleLevel;
        mfScaleFactor = scaleFactor;
        mfLogScaleFactor = logScaleFactor;
        mvScaleFactors = scaleFactorsVec;
        mvLevelSigma2 = levelSigma;
        mvInvLevelSigma2 = invLevelSigma;

    }

    void KeyFrame::InitOutliers ()
    {
        mvbOutlier = vector<bool>(GetIndexedMapPoints().size(),false);
    }

// Default serializing Constructor
    KeyFrame::KeyFrame() :
            mnFrameId(0), mTimeStamp(0.0), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
            mnMaxX(FRAME_MN_MAX_X), mnMinX(FRAME_MN_MIN_Y), mnMaxY (FRAME_MN_MAX_Y), mnMinY (FRAME_MN_MIN_Y),
            mfGridElementWidthInv(0.0), mfGridElementHeightInv(0.0), mbFirstConnection (true), mpParent (static_cast<KeyFrame*> (NULL)),
            mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
            mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
            fx(0.0), fy(0.0), cx(0.0), cy(0.0), invfx(0.0), invfy(0.0),
            mbf(0.0), mb(0.0), mThDepth(0.0), N(0), mnScaleLevels(0), mfScaleFactor(0),
            mfLogScaleFactor(0.0),
             isLoadedFromDisk(true), mbNotErase (false), mbBad (false), mbToBeErased (false)
    {}

    template<class Archive>
    void KeyFrame::serialize(Archive &ar, const unsigned int version) {

        ar & mnId;//Keyframe ID
        //TODO: Uncomment these
        {
            unique_lock<mutex> lock_pose(mMutexPose);
            ar & Tcw;//Pose
        }
        {
            unique_lock<mutex> lock_feature(mMutexFeatures);
            ar & mvpMapPoints; // All map points linked to this keyframe
        }
        {
            unique_lock<mutex> lock_connection(mMutexConnections);
            ar & mbFirstConnection & mpParent & mspChildrens;
        }
/*
//            ar & img;

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////// REMOVED DATA ////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


        //Session data
        // Tracking related vars
        //    ar & mnTrackReferenceForFrame & mnFuseTargetForKF;
        // LocalMaping related vars
        //    ar & mnBALocalForKF & mnBAFixedForKF;
        // KeyFrameDB related vars
        //    ar & mnLoopQuery & mnLoopWords & mLoopScore & mnRelocQuery & mnRelocWords & mRelocScore;
        // LoopClosing related vars
        //    ar & mTcwGBA & mTcwBefGBA & mnBAGlobalForKF;
        ////////////////////////////////// RE-CONSTRUCTIBLE DATA //////////////////////////////////////////////////////////
        ////////////////////////////////////// BAG OF WORDS ///////////////////////////////////////////////////////////////
        // Bow
        //    ar & mBowVec & mFeatVec;
        //    ar & mpKeyFrameDB;
        ////////////////////////////////////////// POSES //////////////////////////////////////////////////////////////////
        // Own poses
        //    ar & Twc & Ow & Cw;
        // Pose relative to parent
        //    ar & mTcp;
        ///////////////////////////////////////// CO-VISIBILITY GRAPH ////////////////////////////////////////////////////
        //    ar & mConnectedKeyFrameWeights & mvpOrderedConnectedKeyFrames & mvOrderedWeights;
        /////////////////////////////////////////// POINTER //////////////////////////////////////////////////////////////
        //    ar & mpMap;
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////// REDUNDANT DATA ////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////// CAMERA & ORB EXTRACTION //////////////////////////////////////////////////////////
        // calibration parameters
        //    ar & const_cast<float &>(fx) & const_cast<float &>(fy) & const_cast<float &>(cx) & const_cast<float &>(cy);
        //    ar & const_cast<float &>(invfx) & const_cast<float &>(invfy) & const_cast<float &>(mbf);
        //    ar & const_cast<float &>(mb) & const_cast<float &>(mThDepth);
        // Scale related
        //    ar & const_cast<int &>(mnScaleLevels) & const_cast<float &>(mfScaleFactor) & const_cast<float &>(mfLogScaleFactor);
        //    ar & const_cast<std::vector<float> &>(mvScaleFactors) & const_cast<std::vector<float> &>(mvLevelSigma2) & const_cast<std::vector<float> &>(mvInvLevelSigma2);
        // Grid related vars
        //    ar & const_cast<int &>(mnGridCols);
        //    ar & const_cast<int &>(mnGridRows);
        //    ar & const_cast<float &>(mfGridElementWidthInv);
        //    ar & const_cast<float &>(mfGridElementHeightInv);
        // Image bounds and calibration
        //    ar & const_cast<int &>(mnMinX) & const_cast<int &>(mnMinY) & const_cast<int &>(mnMaxX) & const_cast<int &>(mnMaxY);
        //    ar & const_cast<cv::Mat &>(mK);
        // Re-constructible variables
        //    ar & const_cast<long unsigned int &>(mnFrameId);
        //    ar & const_cast<double &>(mTimeStamp);
        //    ar & nNextId;
        //    ar & const_cast<int &>(N);
        // KeyPoints, stereo coordinate and descriptors
        //    ar & const_cast<std::vector<cv::KeyPoint> &>(mvKeys);
        //    ar & const_cast<std::vector<cv::KeyPoint> &>(mvKeysUn);
        //    ar & const_cast<std::vector<float> &>(mvuRight);
        //    ar & const_cast<std::vector<float> &>(mvDepth);
        //    ar & const_cast<cv::Mat &>(mDescriptors);
        //  Flags
        //    ar & mbNotErase & mbToBeErased & mbBad & mHalfBaseline;


*/
    }



    void KeyFrame::GetLevels ()
    {
        cout << "Scale levels, mfScaleFactor, mfLogScaleFactor =  " << mnScaleLevels << ", " << mfScaleFactor << ", " << mfLogScaleFactor << endl;
        cout << "mvScaleFactors = ";
        for (float sf : mvScaleFactors)
            cout << sf << ", ";
        cout << endl;
        cout << "mvLevelSigma2 = ";
        for (float sf : mvLevelSigma2)
            cout << sf << ", ";
        cout << endl;
        cout << "mvInvLevelSigma2 = ";
        for (float sf : mvInvLevelSigma2)
            cout << sf << ", ";
        cout << endl;
    }



    void KeyFrame ::AssignGridParameters()
    {
        if (mbNotErase == true || mbToBeErased == true || mbBad == true)
            cout << "Do not erase these" << endl;
        //Reconstruct grid parameters
        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);
        //Assign the value of N
        N = mvpMapPoints.size();
        //Reconstruct other poses from Tcw
        SetPose (Tcw);
        //Reconstruct mTcp (pose relative to parent)
        if (mbBad == true)//Assign the value of mTcp using the parent of the parent keyframe
            mTcp = Tcw * mpParent -> GetPoseInverse();

        return;
    }

    template void KeyFrame::serialize(boost::archive::binary_iarchive &, const unsigned int);

    template void KeyFrame::serialize(boost::archive::binary_oarchive &, const unsigned int);

    template void KeyFrame::serialize(boost::archive::text_oarchive &, const unsigned int);

    template void KeyFrame::serialize(boost::archive::text_iarchive &, const unsigned int);

} //namespace ORB_SLAM
