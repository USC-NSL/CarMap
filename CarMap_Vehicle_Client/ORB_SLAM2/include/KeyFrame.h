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

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"
#include "MapPointObservation.h"
#include "SemanticSegmentor.h"

#include <mutex>
#include "BoostArchiver.h"



namespace ORB_SLAM2
{

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;

class KeyFrame
{
public:

    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB);
    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB, bool isExtraLabels);
    KeyFrame(KeyFrame *kF, Map* pMap, KeyFrameDatabase* pKFDB, cv::Mat);
    KeyFrame(KeyFrame *kF, Map* pMap, KeyFrameDatabase* pKFDB, bool, cv::Mat);

    KeyFrame (unsigned int id, cv::Mat m_Tcw, bool first_connection, KeyFrame* mp_parent, Map* diff_map);

//    KeyFrame ()

    // Pose functions
    void SetPose(const cv::Mat &Tcw);
    cv::Mat GetPose();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter();
    cv::Mat GetStereoCenter();
    cv::Mat GetRotation();
    cv::Mat GetTranslation();

    cv::Mat GetPoseInverseFromPose (const cv::Mat &Tcw_);
    cv::Mat GetPoseFromPoseInverse (const cv::Mat &Twc_);

    void ReconstructKeyFrame (KeyFrameDatabase*, Map*);

    // Bag of Words Representation
    void ComputeBoW();//Construct BoW from current scene (key points)
    void ReconstructBoW ();//Reconstruct BoW from disk (map points)

    // Covisibility graph functions
    void AddConnection(KeyFrame* pKF, const int &weight);
    void EraseConnection(KeyFrame* pKF);
    void UpdateConnections(bool = false);
    //TODO comment these
    void OldUpdateConnections();
    void OldReconstructCoVisibilityGraph ();
    void UpdateBestCovisibles();
    std::set<KeyFrame *> GetConnectedKeyFrames();
    std::vector<KeyFrame* > GetVectorCovisibleKeyFrames();
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);
    std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);
    int GetWeight(KeyFrame* pKF);

    // Spanning tree functions
    void AddChild(KeyFrame* pKF);
    void EraseChild(KeyFrame* pKF);
    void ChangeParent(KeyFrame* pKF);
    std::set<KeyFrame*> GetChilds();
    KeyFrame* GetParent();
    bool hasChild(KeyFrame* pKF);

    // Loop Edges
    void AddLoopEdge(KeyFrame* pKF);
    std::set<KeyFrame*> GetLoopEdges();

    // MapPoint observation functions
    void AddMapPoint(MapPoint* pMP, const size_t &idx);
    void EraseMapPointMatch(const size_t &idx);
    void EraseMapPointMatch(MapPoint* pMP);
    void ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP);
    std::set<MapPoint*> GetMapPoints();
    std::vector<MapPoint*> GetMapPointMatches();
    int TrackedMapPoints(const int &minObs);
    MapPoint* GetMapPoint(const size_t &idx);

    float GetAverageConnectedKeyFrameWeight ();

    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;
    cv::Mat UnprojectStereo(int i);
    cv::Mat UnprojectStereo (float xx, float yy, float z);

    cv::Mat GetDescriptors (void);
    void SetDescriptors (cv::Mat);

    void AssignGridParameters ();

    cv::Mat img;

    void PrintAllMapPoints2DCoordinates ();

    // Semantic segmentation
    void PrintLabelStatistics ();



    // Image
    bool IsInImage(const float &x, const float &y) const;


    //Calculate undistored keypoints from the distorted keypoints list
    void calculateUndistoredKeyPoints (void);

    // Enable/Disable bad flag changes
    void SetNotErase();
    void SetErase();

    // Set/check bad flag
    void SetBadFlag();
    bool isBad();

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp( int a, int b){
        return a>b;
    }

    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2){
        return pKF1->mnId<pKF2->mnId;
    }

    //Assign keyframe database to Keyframe instance
    void SetKeyFrameDB (ORB_SLAM2::KeyFrameDatabase *);

    void SetMapPointer (ORB_SLAM2::Map *);

    void SetCameraCalibration (float *);

    void PrintMapPointIndices ();

    std::vector <MapPoint *> GetIndexedMapPoints ();

    void SyncMapPoints ();

    bool LoadedFromDisk ();

    bool GetFirstConnection ();

    void ReconstructCoVisibilityGraph ();

    void GetLevels ();

    void StoreMapPointObservation ();


    void AssignScaleVariables (float, float, float, std::vector<float>, std::vector<float>, std::vector<float>, std::vector<float>);

    void InitOutliers ();

    void ClearMapPointMatches ();

    void CreateDuplicateMapPoints ();

    vector<MapPoint*> GetOutliers ();

    long unsigned int GetmnId ();

    bool GetIsFirstConnection ();

    bool isBadReconstruction ();

    bool AreMpsLabled ();

//    void InterpolateLabels ();



public:
    // for serialization
    KeyFrame(); // Default constructor for serialization, need to deal with const member
    void SetORBvocabulary(ORBVocabulary *porbv) {mpORBvocabulary=porbv;}
private:
    // serialize is recommended to be private
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version);

    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:
    const int CC_ARRAY_SIZE = 6;
    static long unsigned int nNextId;
    long unsigned int mnId;
    const long unsigned int mnFrameId;

    const int Vocab_Level = 4;

    const double mTimeStamp;

    // Grid (to speed up feature matching)
    const int mnGridCols;
    const int mnGridRows;
    float mfGridElementWidthInv;
    float mfGridElementHeightInv;

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKF;

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;

    //Statistics
    unsigned long int kfDuration;
    unsigned long int pairDuration;
    unsigned long int sortingDuration;
    unsigned long int updateDuration;
    unsigned long int updateBestCovisibles;

    // Variables used by loop closing
    cv::Mat mTcwGBA;
    cv::Mat mTcwBefGBA;
    long unsigned int mnBAGlobalForKF;

    // Calibration parameters
    float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

    // Number of KeyPoints
    int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    const std::vector<cv::KeyPoint> mvKeys;
    //Making changes-KP
    std::vector<cv::KeyPoint> mvKeysUn;

    std::vector <unsigned char> mvLabels;
    const std::vector <unsigned char> mvDPLabels;
    const std::vector <unsigned char> mvMNLabels;
    const std::vector <unsigned char> mvFDPLabels;
    const std::vector <unsigned char> mvGTLabels;


    const std::vector<float> mvuRight; // negative value for monocular points
    const std::vector<float> mvDepth; // negative value for monocular points
    const cv::Mat mDescriptors;

    //BoW
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;

    // Scale
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    std::vector<float> mvScaleFactors;
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;

    // Image bounds and calibration
    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;
    cv::Mat mK;

    //Making changes-KP
    bool isLoadedFromDisk;

    // MapPoints associated to keypoints
    std::vector<MapPoint*> mvpMapPoints;

    std::vector<MapPoint*> dupMvpMPs;

    // Flag to identify outlier associations.
    std::vector<bool> mvbOutlier;

    bool fullyInitialized = false;

    bool mpsLabeled;


    // The following variables need to be accessed trough a mutex to be thread safe.
protected:

    // SE3 Pose and camera center
    cv::Mat Tcw;
    cv::Mat Twc;
    cv::Mat Ow;

    cv::Mat Cw; // Stereo middle point. Only for visualization



    //Vector containing only map points
    std::vector<MapPoint*> indexedMapPoints;

    // BoW
    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBvocabulary;

    // Grid over the image to speed up feature matching
    std::vector< std::vector <std::vector<size_t> > > mGrid;

    std::map<KeyFrame*,int> mConnectedKeyFrameWeights;
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
    std::vector<int> mvOrderedWeights;

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
    KeyFrame* mpParent;
    std::set<KeyFrame*> mspChildrens;
    std::set<KeyFrame*> mspLoopEdges;

    // Bad flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;    

    float mHalfBaseline; // Only for visualization

    Map* mpMap;

    std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
