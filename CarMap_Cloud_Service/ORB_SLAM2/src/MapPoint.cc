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

#include "MapPoint.h"
#include "ORBmatcher.h"
//#include <GlobalDefintions.h>

#include<mutex>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
//#include <include/ZEDInitialization.h>

namespace ORB_SLAM2 {

    long unsigned int MapPoint::nNextId = 0;
    mutex MapPoint::mGlobalMutex;

    MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map *pMap) :
            mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mbTrackInView(false),
            mnTrackReferenceForFrame(0),
            mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
            mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
            mpReplaced(static_cast<MapPoint *>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap) {
        Pos.copyTo(mWorldPos);
        mNormalVector = cv::Mat::zeros(3, 1, CV_32F);


        // map point only exists in this map
        isAddedToBaseMap = false;
        ptrInBaseMap = static_cast<MapPoint*>(NULL);

        // Diff operation
        ResetFromPreviousSession();


        // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
        unique_lock<mutex> lock(mpMap->mMutexPointCreation);
        mnId = nNextId++;
    }


    MapPoint::MapPoint(const cv::Mat &Pos, unsigned char label, KeyFrame *pRefKF, Map *pMap) :
            mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mbTrackInView(false),
            mnTrackReferenceForFrame(0),
            mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
            mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
            mpReplaced(static_cast<MapPoint *>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap) {
        Pos.copyTo(mWorldPos);
        mNormalVector = cv::Mat::zeros(3, 1, CV_32F);

//        cout << "Current label = " << int (label) << endl;
        // Add the label


//        if (pRefKF->GetmnId() % 2 == 0)
//        {
//            label = label;
//        } else
//            label = 255;


        SetSemanticLabel(label);
        AppendLabel(label);



        // Diff operation
        ResetFromPreviousSession();


        // map point only exists in this map
        isAddedToBaseMap = false;
        ptrInBaseMap = static_cast<MapPoint*>(NULL);


        // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
        unique_lock<mutex> lock(mpMap->mMutexPointCreation);
        mnId = nNextId++;
    }



    MapPoint::MapPoint(const cv::Mat &Pos, Map *pMap, Frame *pFrame, const int &idxF) :
            mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mbTrackInView(false), mnTrackReferenceForFrame(0),
            mnLastFrameSeen(0),
            mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
            mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame *>(NULL)), mnVisible(1),
            mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap) {
        Pos.copyTo(mWorldPos);
        cv::Mat Ow = pFrame->GetCameraCenter();
        mNormalVector = mWorldPos - Ow;
        mNormalVector = mNormalVector / cv::norm(mNormalVector);

        cv::Mat PC = Pos - Ow;
        const float dist = cv::norm(PC);
        const int level = pFrame->mvKeysUn[idxF].octave;
        const float levelScaleFactor = pFrame->mvScaleFactors[level];
        const int nLevels = pFrame->mnScaleLevels;

        mfMaxDistance = dist * levelScaleFactor;
        mfMinDistance = mfMaxDistance / pFrame->mvScaleFactors[nLevels - 1];

        pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);


        // map point only exists in this map
        isAddedToBaseMap = false;
        ptrInBaseMap = static_cast<MapPoint*>(NULL);


        // Diff operation
        ResetFromPreviousSession();



        // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
        unique_lock<mutex> lock(mpMap->mMutexPointCreation);
        mnId = nNextId++;
    }


    MapPoint::MapPoint(const cv::Mat &newPos, KeyFrame *pRefKF, Map *pMap, int numberOfObservations, cv::Mat desc) :
            nObs(numberOfObservations),
            mpRefKF (pRefKF),
            mDescriptor (desc),
            mnTrackReferenceForFrame(0), mnFirstKFid (0), mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
            mnCorrectedReference(0), mnBAGlobalForKF(0), mnVisible(1), mnFound(1), mbBad(false),
            mpReplaced(static_cast<MapPoint *>(NULL)), mfMinDistance(0), mfMaxDistance(0),
            mTrackProjX(0), mTrackProjY(0), mTrackProjXR(0), mbTrackInView(0), mnTrackScaleLevel(0),
            mpMap(pMap),
            mTrackViewCos(0)
    {
        mObservations.clear();
        mpObservation.clear();

        SetWorldPos(newPos);

        // Diff operation
        ResetFromPreviousSession();


        // map point only exists in this map
        isAddedToBaseMap = false;
        ptrInBaseMap = static_cast<MapPoint*>(NULL);


        unique_lock<mutex> lock(mpMap->mMutexPointCreation);
        mnId = nNextId++;

    }


    MapPoint::MapPoint(const cv::Mat &newPos, KeyFrame *pRefKF, Map *pMap, int numberOfObservations, cv::Mat desc, unsigned char semanticLabel) :
            nObs(numberOfObservations),
            mpRefKF (pRefKF),
            mDescriptor (desc),
            label(semanticLabel),
            mnTrackReferenceForFrame(0), mnFirstKFid (0), mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
            mnCorrectedReference(0), mnBAGlobalForKF(0), mnVisible(1), mnFound(1), mbBad(false),
            mpReplaced(static_cast<MapPoint *>(NULL)), mfMinDistance(0), mfMaxDistance(0),
            mTrackProjX(0), mTrackProjY(0), mTrackProjXR(0), mbTrackInView(0), mnTrackScaleLevel(0),
            mpMap(pMap),
            mTrackViewCos(0)
    {
        mObservations.clear();
        mpObservation.clear();

        SetWorldPos(newPos);

        // Diff operation
        ResetFromPreviousSession();


        // map point only exists in this map
        isAddedToBaseMap = false;
        ptrInBaseMap = static_cast<MapPoint*>(NULL);


        unique_lock<mutex> lock(mpMap->mMutexPointCreation);
        mnId = nNextId++;

    }


    void MapPoint::SetSemanticLabel(unsigned char setLabel) {
        {
            unique_lock<mutex> lock(mMutexLabel);

            // Error checking
            if (setLabel < 0 || setLabel > SEMANTIC_SEGMENTOR_NUMBER_OF_OBJECTS - 1) {
//                cerr << "Invalid label = " << (int)setLabel << endl;
                label = 255;
            } else {
                label = setLabel;
            }
        }
    }

    unsigned int MapPoint::GetSemanticLabel() {
        {
            unique_lock<mutex> (mMutexLabel);

	    // If the semantic lable does not make any sense
            if (((unsigned int) label < 0) || ((unsigned int) label > SEMANTIC_SEGMENTOR_NUMBER_OF_OBJECTS - 1));
                // do something useful
		    // cerr << (unsigned int) label << " = " << (int) label << endl;
            return (unsigned int) label;
        }
    }

    cv::Scalar MapPoint::GetSemanticLabelColor() {
        unique_lock<mutex> (mMutexLabel);
        return SemanticSegmentor::GetColorCode(label);
    }

    string MapPoint::GetSemanticLabelString() {
        unique_lock<mutex> (mMutexLabel);
        return SemanticSegmentor::GetObjectName(label);
    }



    bool MapPoint::IsPresentInBaseMap ()
    {
        // Has the map-point been added to the base map?
        return isAddedToBaseMap;
    }

    void MapPoint::AddToBaseMap(MapPoint *ptrMP)
    {
        if ( IsPresentInBaseMap() == true ) // is the map point already present in the base map
        {
            cerr << "Map point already present in the base map" << endl;
            return;
        }
        else
        {
            isAddedToBaseMap = true; // mark the map point as already added to the base map
            ptrInBaseMap = ptrMP; // add pointer to the map point in the base map
        }
    }

    MapPoint* MapPoint::GetPtrInBaseMap()
    {
        // Get the pointer to the copy of this map-point in the base map
        if ( IsPresentInBaseMap() == true )
            return ptrInBaseMap;
        else
        {
            cerr << "Map point not added to base map, hence no pointer exists" << endl;
            return static_cast <MapPoint*> (NULL);
        }
    }

    KeyFrame* MapPoint::GetReferenceKeyFrameInBaseMap()
    {
        if ( IsPresentInBaseMap() == true)
        {
            return GetPtrInBaseMap()->GetReferenceKeyFrame();
        }
        else
        {
            cerr << "Map point does not exist in base map, hence no reference keyframe" << endl;
            return static_cast <KeyFrame*> (NULL);
        }
    }


    void MapPoint::ReplaceReferenceKeyFrame (KeyFrame* newRefKF)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        mpRefKF = newRefKF;
    }



    void MapPoint::SetWorldPos(const cv::Mat &Pos) {
        unique_lock<mutex> lock2(mGlobalMutex);
        unique_lock<mutex> lock(mMutexPos);
        Pos.copyTo(mWorldPos);
    }


    cv::Mat MapPoint::GetWorldPos() {
        unique_lock<mutex> lock(mMutexPos);
        return mWorldPos.clone();
    }

    cv::Mat MapPoint::GetNormal() {
        unique_lock<mutex> lock(mMutexPos);
        return mNormalVector.clone();
    }

    KeyFrame *MapPoint::GetReferenceKeyFrame() {
        unique_lock<mutex> lock(mMutexFeatures);
        return mpRefKF;
    }

    void MapPoint::AddMpObservation(KeyFrame *pKF, size_t idx) {
        /*
        //Add new map point observation at pKF at index idx

        MapPoint_Observation newObservation;//temporary variable for storing
        //Load newObservation
        newObservation.Keypoint = pKF->mvKeysUn[idx];//key point
        pKF->mDescriptors.row(idx).copyTo(newObservation.descriptor);//descriptor
        newObservation.depth = pKF->mvDepth[idx];//depth
        newObservation.rightCoordinate = pKF->mvuRight[idx];//right coordinate

        //Load the mpObservation data structure at pKF with the new observation
        mpObservation.insert(std::pair<ORB_SLAM2::KeyFrame *, MapPoint_Observation>(pKF, newObservation));


        //Old data structures
        cv::KeyPoint unKPtoInsert, depth_right_duo;//The keypoints associated with pKF at idx
        cv::Mat currentDescriptor;//Descriptor associated with keypoint


        unKPtoInsert = pKF->mvKeysUn[idx];//key point
        pKF->mDescriptors.row(idx).copyTo(currentDescriptor);//descriptor
        depth_right_duo.pt.x = pKF->mvDepth[idx];//depth
        depth_right_duo.pt.y = pKF->mvuRight[idx];//right coordinate


        //New data structure
        unKP.insert(std::pair<ORB_SLAM2::KeyFrame *, cv::KeyPoint>(pKF, unKPtoInsert));
        descriptorList.insert(std::pair<ORB_SLAM2::KeyFrame *, cv::Mat>(pKF, currentDescriptor));
        depth_right_map.insert(std::pair<ORB_SLAM2::KeyFrame *, cv::KeyPoint>(pKF, depth_right_duo));

        return;

         */}

    void MapPoint::CopyMapPointObservation(MapPoint_Observation *mpObs, KeyFrame *pKF, size_t idx) {
        //Function: copy the contents of the map point observed in Keyframe (pKF) at index (idx) into mpObs

        mpObs->Keypoint = pKF->mvKeysUn[idx];
        pKF->mDescriptors.row(idx).copyTo(mpObs->descriptor);
        mpObs->rightCoordinate = pKF->mvuRight[idx];
        mpObs->depth = pKF->mvDepth[idx];
        return;

    }

    void MapPoint::CopyMpObservation(KeyFrame *pKF, size_t idx, MapPoint_Observation cpObs) {
        mpObservation.insert(std::pair<ORB_SLAM2::KeyFrame *, MapPoint_Observation>(pKF, cpObs));
        return;
    }

    void MapPoint::PrintMpObservation() {
        cout << "Printing mp observation" << endl;
        cout << "Map point mnID = " << mnId << endl;
        cout << "mpObservation size = " << mpObservation.size() << endl;

        for (auto obs:mpObservation) {
            cout << "Keyframe = " << obs.first->mnId << endl;
            cout << "KP = " << obs.second.Keypoint.pt.x << ", " << obs.second.Keypoint.pt.y << endl;
            cout << "Descriptor = " << obs.second.descriptor << endl;
            cout << "Depth and right = " << obs.second.depth << ", " << obs.second.rightCoordinate << endl;

        }

        cout << "Done" << endl;
    }

    void MapPoint::AddObservation(KeyFrame *pKF, size_t idx) {
        //Function: note that THIS map point was also noticed at Keyframe (pKF) at index (idx)
        // to do so, we copy the features of pKF[idx] into mpObservation

        //Copy the features into a MapPoint_Observation mpObs
        MapPoint_Observation mpObs;
        CopyMapPointObservation(&mpObs, pKF, idx);

        //Acquire a lock over the features of the current map point
        unique_lock<mutex> lock(mMutexFeatures);

        //Is the observation already noted?
        if (mObservations.count(pKF))
            return;//If yes, then return
        mObservations[pKF] = idx;//Otherwise, note the index from which the data is being copied


        //If the keyframe, pKF loaded from disk?
        if (pKF->isLoadedFromDisk == false) {
            // If NO, then note the observation mpObs into the list of observations mpObservation for THIS map point
            mpObservation.insert(std::pair<ORB_SLAM2::KeyFrame *, MapPoint_Observation>(pKF, mpObs));
            //AddMpObservation(pKF, idx);//Save data from keypoints in pKF at idx

            if (pKF->mvuRight[idx] >= 0)
                nObs += 2;
            else
                nObs++;
        } else
            cerr << "Shouldn't be here" << endl;

        //Just to make sure we copied the values properly
//        ValidateObservation(mpObservation, pKF, idx);

        return;
    }

    void MapPoint::AddObservation(KeyFrame *pKF, size_t idx, MapPoint_Observation cpObs) {
        //Function: note that THIS map point was ALSO noticed at Keyframe (pKF) at index (idx)

        unique_lock<mutex> lock(mMutexFeatures);
        if (mObservations.count(pKF))
            return;
        mObservations[pKF] = idx;//For pKF, the map point can be found at idx

        mpObservation.insert(std::pair<ORB_SLAM2::KeyFrame *, MapPoint_Observation>(pKF, cpObs));
        if (cpObs.depth >= 0) {
            nObs += 2;
        } else {
            nObs++;
        }
    }

    void MapPoint::EraseObservation(KeyFrame *pKF) {


        bool bBad = false;
        {
            unique_lock<mutex> lock(mMutexFeatures);
            if (mpObservation.count(pKF)) {
                int idx = mObservations[pKF];

                int rightCoordinate;
                if (pKF->isLoadedFromDisk != true)
                    rightCoordinate = pKF->mvuRight[idx];
                else
                    rightCoordinate = mpObservation.find(pKF)->second.rightCoordinate;

                if (rightCoordinate >= 0)
                    nObs -= 2;
                else
                    nObs--;

                mObservations.erase(pKF);
                mpObservation.erase(pKF);

                if (mpRefKF == pKF) {
                    if (mObservations.begin()->first != mpObservation.begin()->first)
                        cerr << "MapPoint::Keyframes do not match" << endl;
                    else
                    {

                        mpRefKF = mpObservation.begin()->first;
                        if (mpRefKF)
                            mnFirstKFid = mpRefKF->mnId;
                    }

                }
//                    mpRefKF=mObservations.begin()->first;

                // If only 2 observations or less, discard point
                if (nObs <= 2)
                    bBad = true;
            }
        }

        if (bBad)
            SetBadFlag();

    }

    map<KeyFrame *, size_t> MapPoint::GetObservations() {
        unique_lock<mutex> lock(mMutexFeatures);
        return mObservations;
    }

    map<KeyFrame *, size_t> MapPoint::GetObservationsUnlocked() {
        unique_lock<mutex> lock(mMutexFeatures);
        return mObservations;
    }

    map<KeyFrame *, MapPoint_Observation> MapPoint::GetMpObservations() {
        unique_lock<mutex> lock(mMutexFeatures);
        return mpObservation;
    }

    int MapPoint::Observations() {
        unique_lock<mutex> lock(mMutexFeatures);
        return nObs;
    }

    void MapPoint::SetBadFlag() {

        map<KeyFrame *, size_t> obs;
        map<KeyFrame *, MapPoint_Observation> mpObs;
        {
            unique_lock<mutex> lock1(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);
            mbBad = true;
            obs = mObservations;
            mpObs = mpObservation;
            mObservations.clear();
            mpObservation.clear();

        }
        for (map<KeyFrame *, size_t>::iterator mit = obs.begin(), mend = obs.end(); mit != mend; mit++) {
            KeyFrame *pKF = mit->first;
//            cout << "This too" << endl;
            pKF->EraseMapPointMatch(mit->second);
        }

//        cout << "This was called" << endl;
        mpMap->EraseMapPoint(this);


    }

    MapPoint *MapPoint::GetReplaced() {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        return mpReplaced;
    }


    void MapPoint::Replace(MapPoint *pMP) {
        if (pMP->mnId == this->mnId)
            return;
        int nvisible, nfound;
        std::map<KeyFrame *, MapPoint_Observation> mpObs;
        mpObs = mpObservation;
        mpObservation.clear();
        map<KeyFrame *, size_t> obs;

        {
            unique_lock<mutex> lock1(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);
            obs = mObservations;
            mObservations.clear();
            mbBad = true;
            nvisible = mnVisible;
            nfound = mnFound;
            mpReplaced = pMP;
        }


        for (map<KeyFrame *, size_t>::iterator mit = obs.begin(), mend = obs.end(); mit != mend; mit++) {
            //Get all observations for the current map point
            // Replace measurement in keyframe
            KeyFrame *pKF = mit->first;
            //Get the corresponding keyframe for the observation
            if (!pMP->IsInKeyFrame(pKF))//If the map point replacement is not present in the keyframe
            {
                pKF->ReplaceMapPointMatch(mit->second, pMP);
                pMP->AddObservation(pKF, mit->second, mpObs.find(pKF)->second);
            } else {
                pKF->EraseMapPointMatch(mit->second);
            }
        }
        pMP->IncreaseFound(nfound);
        pMP->IncreaseVisible(nvisible);
        pMP->ComputeDistinctiveDescriptors();

        mpMap->EraseMapPoint(this);
    }


    unsigned int MapPoint::GetmnId ()
    {
        return mnId;
    }


    bool MapPoint::isBad() {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        return mbBad;
    }

    // Majority voting
    // add a label
    void MapPoint::AppendLabel (unsigned char newLabel)
    {
        this->labelList.push_back (newLabel);
    }

    vector<unsigned char> MapPoint::GetAllLabels (void)
    {
        return this->labelList;
    }

    unsigned char MapPoint::GetConsensusLabel (void)
    {
        int mostFrequentElement [SEMANTIC_SEGMENTOR_NUMBER_OF_OBJECTS] = {0};
        bool hasValidLabel = false;

        if (this->labelList.size() == 0) {
            cerr << "No label" << endl;
            return 255;
        }

        vector<unsigned char> allLabels = GetAllLabels();
        // print everything
        if (GetAllLabels().size()) {
            cout << "MP [" << GetmnId() << "] = [";
            for (unsigned char currentLabel : allLabels)
                cout << SemanticSegmentor::GetObjectName(currentLabel) << ", ";
            cout << " ]" << endl;
        } else;
//            cout << "Empty" << endl;

        for (unsigned char currentLabel : allLabels)
        {
            if ( currentLabel >= SEMANTIC_SEGMENTOR_NUMBER_OF_OBJECTS - 1)
            {
                mostFrequentElement[SEMANTIC_SEGMENTOR_NUMBER_OF_OBJECTS-1]++;
            }
            else
                mostFrequentElement[currentLabel]++;
        }

        int maxElements  = 0;
        int maxElementIndex = 0;

        int otherElements = 0;
        int otherElementIndex = 0;
        bool hasOtherElement = false;
        for (int counter = 0; counter < SEMANTIC_SEGMENTOR_NUMBER_OF_OBJECTS; counter++)
        {
            if (mostFrequentElement[counter] > maxElements)
            {
                if (counter == UNDETECTED_CODE) {
                    maxElementIndex = counter;
                    maxElements = mostFrequentElement[counter];
                }
                else {
                    hasOtherElement = true;
                    maxElementIndex = counter;
                    maxElements = mostFrequentElement[counter];

                    otherElementIndex = counter;
                    otherElements = mostFrequentElement[counter];
                }
            }
        }

        if ( maxElementIndex == UNDETECTED_CODE )
        {
            if (hasOtherElement)
                return otherElementIndex;
            else
                return  maxElementIndex;
        }
        else
            return maxElementIndex;
    }

    void MapPoint::AssignConsensusLabel ()
    {
        this->label = GetConsensusLabel();
    }


    void MapPoint::IncreaseVisible(int n) {
        unique_lock<mutex> lock(mMutexFeatures);
        mnVisible += n;
    }

    void MapPoint::IncreaseFound(int n) {
        unique_lock<mutex> lock(mMutexFeatures);
        mnFound += n;
    }

    float MapPoint::GetFoundRatio() {
        unique_lock<mutex> lock(mMutexFeatures);
        return static_cast<float>(mnFound) / mnVisible;
    }

    void MapPoint::ComputeDistinctiveDescriptors() {

        //Function: Compute a DISTINCTIVE descriptor from all the map point observations
        //Procedure:
        // Retrieve all observed descriptors
        // Compute distances between them and take one with the least median distance to the rest


        // Retrieve all the descriptors
        vector<cv::Mat> vDescriptors;
        map<KeyFrame *, MapPoint_Observation> mpobservations;
        map<KeyFrame *, size_t> observations;
        {
            unique_lock<mutex> lock1(mMutexFeatures);
            if (mbBad)
                return;
            observations = mObservations;
            mpobservations = mpObservation;
        }

        //Return if there are no observations
        if (mpobservations.empty())
            return;

        //Iterate through all the observations and store them in vDescriptors
        //Get the number of map point observations that have a descriptor
        vDescriptors.reserve(GetNumberOfMpDescriptors(mpobservations));

        //Iterate through all map point observations and insert them in vDescriptors
        for (map<KeyFrame *, MapPoint_Observation>::iterator mit = mpobservations.begin(), mend = mpobservations.end();
             mit != mend; mit++) {
            //Skip if there is no descriptor for map point observation
            if (mit->second.isLoadedFromDisk() == true)
                continue;

            KeyFrame *pKF = mit->first;
            if (!pKF->isBad()) {
                if (pKF->isLoadedFromDisk == true)//if loaded from disk, take descriptor from mpObservation
                {
                    cv::Mat diskDescriptor = mpObservation.find(pKF)->second.descriptor;
                    vDescriptors.push_back(diskDescriptor);
                } else//if pKF from current session, load from mDescriptors
                    vDescriptors.push_back(pKF->mDescriptors.row(observations.find(pKF)->second));
            }
        }

        if (vDescriptors.empty())
            return;

        // Compute distances between them
        const size_t N = vDescriptors.size();

        float Distances[N][N];
        for (size_t i = 0; i < N; i++) {
            Distances[i][i] = 0;
            for (size_t j = i + 1; j < N; j++) {
                int distij = ORBmatcher::DescriptorDistance(vDescriptors[i], vDescriptors[j]);
                Distances[i][j] = distij;
                Distances[j][i] = distij;
            }
        }

        // Take the descriptor with least median distance to the rest
        int BestMedian = INT_MAX;
        int BestIdx = 0;
        for (size_t i = 0; i < N; i++) {
            vector<int> vDists(Distances[i], Distances[i] + N);
            sort(vDists.begin(), vDists.end());
            int median = vDists[0.5 * (N - 1)];

            if (median < BestMedian) {
                BestMedian = median;
                BestIdx = i;
            }
        }

        {
            unique_lock<mutex> lock(mMutexFeatures);
            mDescriptor = vDescriptors[BestIdx].clone();
        }
    }

    int MapPoint::GetNumberOfMpDescriptors(std::map<KeyFrame *, MapPoint_Observation> mpObs) {
        int descriptorObs = 0;
        for (std::pair<KeyFrame *, MapPoint_Observation> ob : mpObs) {
            if (ob.second.isLoadedFromDisk() == false)
                descriptorObs++;
        }

        return descriptorObs;
    }

    cv::Mat MapPoint::GetDescriptor() {
        unique_lock<mutex> lock(mMutexFeatures);
        return mDescriptor.clone();
    }


    cv::Mat MapPoint::GetBestDescriptor(KeyFrame *pKF) {
        //Is the mp observation for this map point at pKF loaded from disk?
        if (this->GetMpObservations().find(pKF)->second.isLoadedFromDisk() == true) {
//            cout << "pMP [kF] " << mnId << "[" << pKF->mnId << "]" << " is loaded from disk" << endl;
//            cout << "Descriptor = " << GetDescriptor() << endl;
//            return this->GetMpObservations().find(pKF)->second.descriptor;
            return GetDescriptor(); // if yes, then return distinctive descriptor
        }
        else {// else, return the mp observation descriptor
//            cout << "pMP [kF] " << mnId << "[" << pKF->mnId << "]" << " is NOT loaded from disk" << endl;
//            cout << "Descriptor = " << this->GetMpObservations().find(pKF)->second.GetDescriptor() << endl;

//            return GetDescriptor();
            return GetDescriptor();
//            return this->GetMpObservations().find(pKF)->second.GetDescriptor();
        }

    }

    int MapPoint::GetIndexInKeyFrame(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexFeatures);
        if (mObservations.count(pKF))
            return mObservations[pKF];
        else
            return -1;
    }

    bool MapPoint::IsInKeyFrame(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexFeatures);
        return (mObservations.count(pKF));
    }

    void MapPoint::UpdateNormalAndDepth() {
        map<KeyFrame *, size_t> observations;
        KeyFrame *pRefKF;
        cv::Mat Pos;
        {
            unique_lock<mutex> lock1(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);
            if (mbBad)
                return;
            observations = mObservations;
            pRefKF = mpRefKF;
            Pos = mWorldPos.clone();
        }

        if (observations.empty())
            return;

        cv::Mat normal = cv::Mat::zeros(3, 1, CV_32F);
        int n = 0;
        for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end();
             mit != mend; mit++) {
            KeyFrame *pKF = mit->first;
            cv::Mat Owi = pKF->GetCameraCenter();
            cv::Mat normali = mWorldPos - Owi;
            normal = normal + normali / cv::norm(normali);
            n++;
        }

        if (pRefKF == NULL)
        {
            cerr << "Error: MapPoint: UpdateNormalAndDepth () ---- No reference key frame for mp ID = " << this->mnId << endl;
            return;
        }
        cv::Mat PC = Pos - pRefKF->GetCameraCenter();
        const float dist = cv::norm(PC);
        //Making changes-KP
//        cv::KeyPoint defaultKP = unKP.find(pRefKF)->second;
        cv::KeyPoint mpKP = mpObservation.find(pRefKF)->second.Keypoint;
//        cv::KeyPoint defaultKP = pRefKF->mvKeysUn[observations[pRefKF]];



//        const int level = unKP.find(pRefKF)->second.octave;
        const int level = mpKP.octave;
//        const int level1 = defaultKP.octave;
//
//        if (level != level1) {
//            cout << "KP problem here" << endl;
//            cout << "Default = " << defaultKP.pt.x << ", " << defaultKP.pt.y << endl;
//            cout << "Our = " << mpKP.pt.x << ", " << mpKP.pt.y << endl;
//        }


        const float levelScaleFactor = pRefKF->mvScaleFactors[level];
        const int nLevels = pRefKF->mnScaleLevels;

        {
            unique_lock<mutex> lock3(mMutexPos);
            mfMaxDistance = dist * levelScaleFactor;
            mfMinDistance = mfMaxDistance / pRefKF->mvScaleFactors[nLevels - 1];
            mNormalVector = normal / n;
        }
    }

    float MapPoint::GetMinDistanceInvariance() {
        unique_lock<mutex> lock(mMutexPos);
        return 0.8f * mfMinDistance;
    }

    float MapPoint::GetMaxDistanceInvariance() {
        unique_lock<mutex> lock(mMutexPos);
        return 1.2f * mfMaxDistance;
    }

    int MapPoint::PredictScale(const float &currentDist, KeyFrame *pKF) {
        float ratio;
        {
            unique_lock<mutex> lock(mMutexPos);
            ratio = mfMaxDistance / currentDist;
        }

        int nScale = ceil(log(ratio) / pKF->mfLogScaleFactor);
        if (nScale < 0)
            nScale = 0;
        else if (nScale >= pKF->mnScaleLevels)
            nScale = pKF->mnScaleLevels - 1;

        return nScale;
    }

    // CUSTOM FUNCTIONS

    bool PRINT_EVERYTHING = false;

    void MapPoint::ValidateObservation(std::map<ORB_SLAM2::KeyFrame *, MapPoint_Observation> mpObs, KeyFrame *kf,
                                       size_t index) {
        if (mpObs.find(kf)->first->mnId != kf->mnId)
            cerr << "Error in matching keyframe IDs in ValidateObservation" << endl;
        else {

            if (!CompareKP(mpObs.find(kf)->second, kf->mvKeysUn[index]))
                cerr << "KPs do not match" << endl;
            if (!CompareRightDepth(mpObs.find(kf)->second, kf->mvDepth[index], kf->mvuRight[index]))
                cerr << "Right and depth match do not match" << endl;
            if (!CompareDescriptor(mpObs.find(kf)->second, kf->mDescriptors.row(index)))
                cerr << "Descriptors do not match" << endl;
        }
    }


    int MapPoint::PredictScale(const float &currentDist, Frame *pF) {
        float ratio;
        {
            unique_lock<mutex> lock(mMutexPos);
            ratio = mfMaxDistance / currentDist;
        }

        int nScale = ceil(log(ratio) / pF->mfLogScaleFactor);
        if (nScale < 0)
            nScale = 0;
        else if (nScale >= pF->mnScaleLevels)
            nScale = pF->mnScaleLevels - 1;

        return nScale;
    }

    void MapPoint::SetMapPointer(ORB_SLAM2::Map *ptrToMap) {
        mpMap = ptrToMap;
    }

    void MapPoint::AppendObservations (KeyFrame *pKF, size_t index)
    {
        mObservations.insert(make_pair(pKF, index));
    }

    void MapPoint::SetFromPreviousSession ()
    {
        fromPreviousSession = true;
        numberOfOccurencesInNewSession = 0;

    }

    void MapPoint::ResetFromPreviousSession ()
    {
        fromPreviousSession = false;
    }

    bool MapPoint::GetFromPreviousSession ()
    {
        return fromPreviousSession;
    }

    void MapPoint::IncrementNumberOfOccurencesInNewSession ()
    {
        numberOfOccurencesInNewSession++;
    }

    unsigned int MapPoint::GetNumberOfOccurencesInNewSession ()
    {
        return numberOfOccurencesInNewSession;
    }



    void MapPoint::UpdateMpObservationIndex (KeyFrame *pKF, size_t id)
    {

//        cout << "Add ID = " << id << " to pKF = " << pKF->mnId << endl;

        std::map<KeyFrame*, MapPoint_Observation>::iterator it = mpObservation.find(pKF);
        if (it != mpObservation.end())
            (*it).second.index = id;

//        cout << "pKF = " << pKF->mnId << ", id = " << mpObservation.find(pKF)->second.index << endl;


//        MapPoint_Observation oldObs = mpObservation.find(pKF)->second;
//        oldObs.index = id;
//        mpObservation.insert(std::pair<ORB_SLAM2::KeyFrame *, MapPoint_Observation>(pKF, oldObs));
//        cout << oldObs.index << endl;
//        MapPoint_Observation oldObs = GetMpObservations().at(pKF);
//        oldObs.index = id;
//
//        mpObservation.insert(make_pair(pKF, oldObs));
        return;

    }

    // Serialization constructor
    MapPoint::MapPoint() :
            nObs(0), mnTrackReferenceForFrame(0), mpRefKF (static_cast<KeyFrame*>(NULL)), mnFirstKFid (0),
            mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
            mnCorrectedReference(0), mnBAGlobalForKF(0), mnVisible(1), mnFound(1), mbBad(false),
            mpReplaced(static_cast<MapPoint *>(NULL)), mfMinDistance(0), mfMaxDistance(0),
            mTrackProjX(0), mTrackProjY(0), mTrackProjXR(0), mbTrackInView(0), mnTrackScaleLevel(0),
            mTrackViewCos(0)
    {
        //TODO uncomment this
        mObservations.clear(); // clear all the observations because they will be reconstructed
        mpObservation.clear();
        isAddedToBaseMap = false;
        ptrInBaseMap = static_cast <MapPoint*> (NULL);

        // Diff operation
        SetFromPreviousSession(); // Flag this as from previous session
    }



    template<class Archive>
    void MapPoint::serialize(Archive &ar, const unsigned int version) {

        // save/load map point ID
        {
            ar & mnId;
        }
        // save/load number of observations and the descriptor
        {
            unique_lock<mutex> lock(mMutexFeatures);
            ar & nObs;
            ar & mDescriptor;//Distinctive descriptor of map point
        }
        // save/load the 3D position
        {
            unique_lock<mutex> lock(mMutexPos);
            ar & mWorldPos;//World position of map point
        }
        // save/load the reference keyframe
        {
            unique_lock<mutex> lock1(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);
            ar & mpRefKF;//Reference keyframe for map point
        }
        // save/load the label for the keyframe
        {
            unique_lock<mutex> (mMutexLabel);
            ar & label;
        }
//        ar & mpObservation;

        //TODO remove this later
//        ar & mObservations;

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////// REMOVED DATA ////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        //Session data
        // Local Mapping related vars
        //    ar & mnBALocalForKF & mnFuseCandidateForKF;
        // Loop Closing related vars
        //    ar & mnLoopPointForKF & mnCorrectedByKF & mnCorrectedReference & mPosGBA & mnBAGlobalForKF;


        //Reconstructible data
            //    ar & mNormalVector;
        //Pointer
            //    ar & mpMap;
        // Normal vector and map point observations
            //    ar & nNormalVector;
            //    ar & mObservation;


        //Redundant data
        // Tracking variables
            //        ar & mTrackProjX;
            //        ar & mTrackProjY;
            //        ar & mTrackProjXR;
            //        ar & mbTrackInView;
            //        ar & mnTrackScaleLevel;
            //        ar & mTrackViewCos;
            //        ar & mnTrackReferenceForFrame;
            //        ar & mnLastFrameSeen;

        //        ar & mfMinDistance & mfMaxDistance;


    }

    template void MapPoint::serialize(boost::archive::binary_iarchive &, const unsigned int);

    template void MapPoint::serialize(boost::archive::binary_oarchive &, const unsigned int);

    template void MapPoint::serialize(boost::archive::text_oarchive &, const unsigned int);

    template void MapPoint::serialize(boost::archive::text_iarchive &, const unsigned int);


    void MapPoint::ComputeKeyPoints(KeyFrame *pKF, const int index) {
        /*

        cv::KeyPoint unKPtoInsert, depth_right_duo;
        cv::Mat currentDescriptor;


        MapPoint_Observation recentObservation;

        if (pKF->isLoadedFromDisk == false) {
            unKPtoInsert = pKF->mvKeysUn[index];
            pKF->mDescriptors.row(index).copyTo(currentDescriptor);
            depth_right_duo.pt.x = pKF->mvDepth[index];
            depth_right_duo.pt.y = pKF->mvuRight[index];


            //New data structure
            recentObservation.Keypoint = pKF->mvKeysUn[index];
            pKF->mDescriptors.row(index).copyTo(recentObservation.descriptor);
            recentObservation.depth = pKF->mvDepth[index];
            recentObservation.rightCoordinate = pKF->mvDepth[index];
        } else {
            unKPtoInsert = pKF->GetMapPoint(index)->unKP.find(pKF)->second;
            pKF->GetMapPoint(index)->descriptorList.find(pKF)->second.copyTo(currentDescriptor);
            depth_right_duo = pKF->GetMapPoint(index)->depth_right_map.find(pKF)->second;

            recentObservation.Keypoint = pKF->GetMapPoint(index)->mpObservation.find(pKF)->second.Keypoint;
            pKF->GetMapPoint(index)->mpObservation.find(pKF)->second.descriptor.copyTo(currentDescriptor);
            recentObservation.depth = pKF->GetMapPoint(index)->mpObservation.find(pKF)->second.depth;
            recentObservation.rightCoordinate = pKF->GetMapPoint(index)->mpObservation.find(
                    pKF)->second.rightCoordinate;

        }


        mpObservation.insert(std::pair<ORB_SLAM2::KeyFrame *, MapPoint_Observation>(pKF, recentObservation));
        mObservations.insert(std::pair<ORB_SLAM2::KeyFrame *, size_t>(pKF, index));
        unKP.insert(std::pair<ORB_SLAM2::KeyFrame *, cv::KeyPoint>(pKF, unKPtoInsert));
        descriptorList.insert(std::pair<ORB_SLAM2::KeyFrame *, cv::Mat>(pKF, currentDescriptor));
        depth_right_map.insert(std::pair<ORB_SLAM2::KeyFrame *, cv::KeyPoint>(pKF, depth_right_duo));

         */}

    void MapPoint::CompareWholeDS(MapPoint_Observation mp, cv::KeyPoint kp, cv::Mat desc, float depth, float right) {
        CompareKP(mp, kp);
        CompareDescriptor(mp, desc);
        CompareRightDepth(mp, depth, right);
    }

    bool MapPoint::CompareKP(MapPoint_Observation mp, cv::KeyPoint kp) {
        if (PRINT_EVERYTHING) {
            cout << mp.Keypoint.pt.x << ", " << mp.Keypoint.pt.y << " | " << kp.pt.x << ", " << kp.pt.y << endl;
        }
        if ((mp.Keypoint.pt.x == kp.pt.x) && (mp.Keypoint.pt.y == kp.pt.y)) {
            return true;
        } else
            cerr << "Error in Compare KP" << endl;

        return false;
    }

    void MapPoint::AddCoordinates (KeyFrame * pKF, float x, float y, float z, float r, int index)
    {
        MapPoint_Observation mpObs;
        mpObs.Init_MpObs();
        mpObs.Keypoint.pt.x = x;
        mpObs.Keypoint.pt.y = y;
        mpObs.depth = z;
        mpObs.rightCoordinate = r;
        mpObs.index = index;

//        cout << "(mnid, x, y, z, r) = (" << mnId << ", " << x << ", " << y << ", " << z << ", " << r << " )" << endl;

        mpObservation.insert( std::pair <KeyFrame * , MapPoint_Observation> (pKF, mpObs));


//        mpObservation.at(pKF).Keypoint.pt.x = x;
//        mpObservation.at(pKF).Keypoint.pt.y = y;
//        mpObservation.at(pKF).depth = z;
//        mpObservation.at(pKF).rightCoordinate = r;

        return;
    }



    void MapPoint::CompareMpObsAndDetKF ()
    {

        size_t mpObsSize = mpObservation.size();
        size_t detectionSize = detectionKeyFrames.size();

        if (mpObsSize != detectionSize)
        {
            cerr << "mp = " << mnId << ", mp " << mpObsSize << " != " << detectionSize << endl;
        }
        else
        {

            bool print = false;
            for (std::pair <KeyFrame*, MapPoint_Observation> mpKf : mpObservation)
            {
                if ( detectionKeyFrames.find ( mpKf.first ) == detectionKeyFrames.end() )
                    print = true;
            }


            if (print) {
                cout << endl << "mpObservations = ";
                for (std::pair<KeyFrame *, MapPoint_Observation> mpKf : mpObservation)
                    cout << mpKf.first->mnId << ", ";

                cout << endl << "detectionKeyFrames = ";
                for ( std::pair <KeyFrame*, size_t> kf : detectionKeyFrames)
                    cout << kf.first->mnId << ", ";
                cout << endl;
            } else
            {
                cout << endl << "mpObservations = ";
                for (std::pair<KeyFrame *, MapPoint_Observation> mpKf : mpObservation)
                    cout << mpKf.first->mnId << ", ";

                cout << endl << "detectionKeyFrames = ";
                for ( std::pair <KeyFrame*, size_t> kf : detectionKeyFrames)
                    cout << kf.first->mnId << ", ";
                cout << endl;
            }
//                cout << "perfect for mp = " << mnId << endl;
        }


    }

//    void MapPoint::ReconstructMpObservation(KeyFrame *rcKF)
//    {
//        // Find the position of this map point in the camera coordinate system
//        // I.e., transform from world coordinates to xCamPos, yCamPos, and zDepth
//
//        // Find the reference keyframe in which the map-point was detected
//
//            KeyFrame *kf = rcKF;
//            if (!kf) {
//                cerr << "NO ref KF for mp = " << mnId << endl;
//                return;
//            }
//
//
//            // Get the transform of the keyframe
//            cv::Mat kfRot = kf->GetRotation().t();//Rcw pose of the reference keyframe
//            cv::Mat mpPose = GetWorldPos();//World position of this map point
//            cv::Mat kfCC = kf->GetCameraCenter();//Camera center of reference keyframe
//
//            // Using the keyframe and the camera's transform
//            // Do an inverse transformation to get its position in the camera coordinate system
//            cv::Mat mpPosition = kfRot.inv() * (mpPose - kfCC);
//
//
//            float xPos = mpPosition.at<float>(0, 0);
//            float yPos = mpPosition.at<float>(1, 0);
//            float zPos = mpPosition.at<float>(2, 0);
//
//            float xCamPos = (xPos / (kf->invfx * zPos)) + kf->cx;
//            float yCamPos = (yPos / (kf->invfy * zPos)) + kf->cy;
//
//            // Find xPos in the right eye of the stereo camera
//            float mbf = kf->mbf;
//            float disparity = mbf / zPos;
//            float rc = xCamPos - disparity;
//
//            // Add coordinates to the map point
//            AddCoordinates(kf, xCamPos, yCamPos, zPos, rc, kfPair.second);
//        return;
//    }



    void MapPoint::ReconstructMpObservation()
    {
        // Find the position of this map point in the camera coordinate system
        // I.e., transform from world coordinates to xCamPos, yCamPos, and zDepth

        // Find the reference keyframe in which the map-point was detected
        for ( std::pair <KeyFrame*, size_t> kfPair : detectionKeyFrames) {
            KeyFrame *kf = kfPair.first;
            if (!kf) {
                cerr << "NO ref KF for mp = " << mnId << endl;
                continue;
            }


            // Get the transform of the keyframe
            cv::Mat kfRot = kf->GetRotation().t();//Rcw pose of the reference keyframe
            cv::Mat mpPose = GetWorldPos();//World position of this map point
            cv::Mat kfCC = kf->GetCameraCenter();//Camera center of reference keyframe

            // Using the keyframe and the camera's transform
            // Do an inverse transformation to get its position in the camera coordinate system
            cv::Mat mpPosition = kfRot.inv() * (mpPose - kfCC);


            float xPos = mpPosition.at<float>(0, 0);
            float yPos = mpPosition.at<float>(1, 0);
            float zPos = mpPosition.at<float>(2, 0);

            float xCamPos = (xPos / (kf->invfx * zPos)) + kf->cx;
            float yCamPos = (yPos / (kf->invfy * zPos)) + kf->cy;

            // Find xPos in the right eye of the stereo camera
            float mbf = kf->mbf;
            float disparity = mbf / zPos;
            float rc = xCamPos - disparity;


            // Add coordinates to the map point
            AddCoordinates(kf, xCamPos, yCamPos, zPos, rc, kfPair.second);
        }

        return;
    }


    bool MapPoint::CompareDescriptor(MapPoint_Observation mp, cv::Mat desc) {
        if (PRINT_EVERYTHING) {
            cout << "mp " << mp.descriptor << endl;
            cout << "ob " << desc << endl;
        }
        ORB_SLAM2::ORBmatcher matcher;
        if (matcher.DescriptorDistance(mp.descriptor, desc) != 0) {
            cerr << "Error in compare descriptors" << endl;
            return false;
        } else
            return true;
    }

    bool MapPoint::CompareRightDepth(MapPoint_Observation mp, float depth, float right) {
        if (PRINT_EVERYTHING) {
            cout << mp.depth << ", " << mp.rightCoordinate << " | " << depth << ", " << right << endl;
        }
        if ((mp.depth == depth) && (mp.rightCoordinate == right))
            return true;
        else
            cerr << "Problem in compare depth and right" << endl;
        return false;
    }


} //namespace ORB_SLAM
