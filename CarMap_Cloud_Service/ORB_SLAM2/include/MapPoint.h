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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"
#include "MapPointObservation.h"
#include "SemanticSegmentor.h"
//#include <GlobalDefintions.h>

#include<opencv2/core/core.hpp>
#include<mutex>
#include "BoostArchiver.h"
//#include <ZEDInitialization.h>

namespace ORB_SLAM2
{

class KeyFrame;
class Map;
class Frame;



class MapPoint
{
public:
    MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);//constructor
    MapPoint(const cv::Mat &Pos, unsigned char label, KeyFrame* pRefKF, Map* pMap);//constructor
    MapPoint(const cv::Mat &Pos,  Map* pMap, Frame* pFrame, const int &idxF);//constructor
    MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map *pMap, int numberOfObservations, cv::Mat desc);//constructor
    MapPoint(const cv::Mat &newPos, KeyFrame *pRefKF, Map *pMap, int numberOfObservations, cv::Mat desc, unsigned char semanticLabel);

    void SetWorldPos(const cv::Mat &Pos);
    cv::Mat GetWorldPos();

    cv::Mat GetNormal();
    KeyFrame* GetReferenceKeyFrame();

    std::map<KeyFrame*,size_t> GetObservations();
    int Observations();

    void AddObservation(KeyFrame* pKF,size_t idx);
    void AddObservation (KeyFrame* pKF, size_t idx, MapPoint_Observation cpObs);
    void EraseObservation(KeyFrame* pKF);

    int GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(MapPoint* pMP);    
    MapPoint* GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    int PredictScale(const float &currentDist, KeyFrame*pKF);
    int PredictScale(const float &currentDist, Frame* pF);

    void ComputeKeyPoints(KeyFrame* pKF, const int index);
    void RemoveKeyPointAssociation (KeyFrame* pKF);

    void SetMapPointer (ORB_SLAM2::Map *);

    void AddMpObservation (KeyFrame*, size_t);
    void CopyMpObservation (KeyFrame*, size_t, MapPoint_Observation);
    void CopyMapPointObservation (MapPoint_Observation* , KeyFrame*, size_t);
    map<KeyFrame*, MapPoint_Observation> GetMpObservations ();

    int GetNumberOfMpDescriptors (std::map <KeyFrame*, MapPoint_Observation>);

    cv::Mat GetBestDescriptor (KeyFrame *);


    map<KeyFrame*, size_t> GetObservationsUnlocked();

    void PrintMpObservation ();


    void CompareWholeDS (MapPoint_Observation mp, cv::KeyPoint kp, cv::Mat desc, float depth, float right);
    bool CompareKP (MapPoint_Observation mp, cv::KeyPoint kp);
    bool CompareDescriptor (MapPoint_Observation mp, cv::Mat desc);
    bool CompareRightDepth (MapPoint_Observation mp, float depth, float right);

    // Semantic label
    unsigned int GetSemanticLabel ();
    void SetSemanticLabel (unsigned char);
    string GetSemanticLabelString ();
    cv::Scalar GetSemanticLabelColor ();

    // Diff operation
    bool GetFromPreviousSession ();
    void SetFromPreviousSession ();
    void ResetFromPreviousSession ();

    void IncrementNumberOfOccurencesInNewSession ();
    unsigned int GetNumberOfOccurencesInNewSession ();



    void ValidateObservation(std::map<ORB_SLAM2::KeyFrame *, MapPoint_Observation> mpObs, KeyFrame *kf, size_t index);

    void AppendObservations (KeyFrame * pKF, size_t index);

    void UpdateMpObservationIndex (KeyFrame*, size_t);

    void ReconstructMpObservation ();
    void ReconstructMpObservation (KeyFrame *);
    void Recontruct2DCameraCoordinates ();

    void AddCoordinates (KeyFrame *pKF, float x, float y, float z, float r, int index);
    void CompareMpObsAndDetKF ();

    void AddToBaseMap (MapPoint *);//Add the map point to the base map
    bool IsPresentInBaseMap ();//Is the map point present in the base map
    MapPoint *GetPtrInBaseMap ();//Get the pointer for this map point in the base map
    KeyFrame *GetReferenceKeyFrameInBaseMap ();//Get the reference keyframe for this map point in the base map
    void ReplaceReferenceKeyFrame (KeyFrame*); //Replace the reference keyframe of this map point in the base map


    // Majority voting for labels
    void AppendLabel (unsigned char);
    vector<unsigned char> GetAllLabels (void);
    unsigned char GetConsensusLabel (void);
    void AssignConsensusLabel (void);

    unsigned int GetmnId ();


public:
    // for serialization
    MapPoint();
private:
    // serialize is recommended to be private
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version);

public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    unsigned long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;

    // semantic segmentation


    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackProjXR;
    bool mbTrackInView;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;    
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;


    static std::mutex mGlobalMutex;

    //Improvements
    std::map <ORB_SLAM2::KeyFrame*, cv::KeyPoint> KP;
    std::map <ORB_SLAM2::KeyFrame*, cv::KeyPoint> unKP;
    std::map <ORB_SLAM2::KeyFrame*, cv::KeyPoint> depth_right_map;
    std::map <ORB_SLAM2::KeyFrame*, cv::Mat> descriptorList;

    vector <unsigned char> labelList;


    //Integration
    std::map <ORB_SLAM2::KeyFrame*, MapPoint_Observation> mpObservation;
        std::map <KeyFrame *, size_t > detectionKeyFrames;


        // Feature map stitching
        //flag to indicate whether this map point has been added to the base map or not
        bool isAddedToBaseMap;
        // pointer to copy of this map point in the base map
        MapPoint *ptrInBaseMap;


protected:    

     // Position in absolute coordinates
     cv::Mat mWorldPos;



    // Semantic segmentation
    unsigned char label;

    // diff operation
    bool fromPreviousSession;
    unsigned int numberOfOccurencesInNewSession;

     // Keyframes observing the point and associated index in keyframe
     std::map<KeyFrame*,size_t> mObservations;

     // Mean viewing direction
     cv::Mat mNormalVector;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;

     // Reference KeyFrame
     KeyFrame* mpRefKF;

     // Tracking counters
     int mnVisible;
     int mnFound;

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;
     MapPoint* mpReplaced;

     // Scale invariance distances
     float mfMinDistance;
     float mfMaxDistance;

     Map* mpMap;

     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
    std::mutex mMutexLabel;
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
