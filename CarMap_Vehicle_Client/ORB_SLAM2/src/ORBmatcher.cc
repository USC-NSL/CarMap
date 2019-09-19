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

#include "ORBmatcher.h"

#include<limits.h>

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include<stdint-gcc.h>

using namespace std;

namespace ORB_SLAM2 {

    const int ORBmatcher::TH_HIGH = 100;
    const int ORBmatcher::TH_LOW = 50;
    const int ORBmatcher::HISTO_LENGTH = 30;

    // Constructor
    ORBmatcher::ORBmatcher(float nnratio, bool majorityVoting, bool checkOri) : mfNNratio(nnratio), mbCheckOrientation(checkOri), majorityVoting(majorityVoting) {
    }

//    ORBmatcher::ORBmatcher(float nnratio, bool majorityVoting, bool checkOri) :
//    majorityVoting (majorityVoting), mfNNratio(nnratio), mbCheckOrientation(checkOri) {
//    }


    // Project map point vpMapPoints in the Frame F and try to do feature matching
    int ORBmatcher::SearchByProjection(Frame &F, const vector<MapPoint *> &vpMapPoints, const float th)
    {
        int nmatches = 0;
        unsigned int prevSessionMatches = 0;
        const bool isDebugMode = false;
        const bool bFactor = th != 1.0;

        for (size_t iMP = 0; iMP < vpMapPoints.size(); iMP++) {
            MapPoint *pMP = vpMapPoints[iMP];
            if (!pMP->mbTrackInView)
                continue;

            if (pMP->isBad())
                continue;

            const int &nPredictedLevel = pMP->mnTrackScaleLevel;

            // The size of the window will depend on the viewing direction
            float r = RadiusByViewingCos(pMP->mTrackViewCos);

            if (bFactor)
                r *= th;

            const vector<size_t> vIndices =
                    F.GetFeaturesInArea(pMP->mTrackProjX, pMP->mTrackProjY, r * F.mvScaleFactors[nPredictedLevel],
                                        nPredictedLevel - 1, nPredictedLevel);

            if (vIndices.empty())
                continue;

            const cv::Mat MPdescriptor = pMP->GetDescriptor();

            int bestDist = 256;
            int bestLevel = -1;
            int bestDist2 = 256;
            int bestLevel2 = -1;
            int bestIdx = -1;

            // Get best and second matches with near keypoints
            for (vector<size_t>::const_iterator vit = vIndices.begin(), vend = vIndices.end(); vit != vend; vit++) {
                const size_t idx = *vit;

                if (F.mvpMapPoints[idx])
                    if (F.mvpMapPoints[idx]->Observations() > 0)
                        continue;

                if (F.mvuRight[idx] > 0) {
                    const float er = fabs(pMP->mTrackProjXR - F.mvuRight[idx]);
                    if (er > r * F.mvScaleFactors[nPredictedLevel])
                        continue;
                }

                const cv::Mat &d = F.mDescriptors.row(idx);

                const int dist = DescriptorDistance(MPdescriptor, d);

                if (dist < bestDist) {
                    bestDist2 = bestDist;
                    bestDist = dist;
                    bestLevel2 = bestLevel;
                    bestLevel = F.mvKeysUn[idx].octave;
                    bestIdx = idx;
                } else if (dist < bestDist2) {
                    bestLevel2 = F.mvKeysUn[idx].octave;
                    bestDist2 = dist;
                }
            }

            // Apply ratio to second match (only if best and second are in the same scale level)
            if (bestDist <= TH_HIGH) {
                if (bestLevel == bestLevel2 && bestDist > mfNNratio * bestDist2)
                    continue;

                F.mvpMapPoints[bestIdx] = pMP;
                nmatches++;

                // append the label of this map-point
                if (majorityVoting == true)
                    pMP->AppendLabel(F.mvLabels[bestIdx]);

                if (pMP->GetFromPreviousSession() == true)
                {
                    pMP->IncrementNumberOfOccurencesInNewSession();
                    prevSessionMatches++;
                }
            }
        }

        return nmatches;
    }

    float ORBmatcher::RadiusByViewingCos(const float &viewCos) {
        if (viewCos > 0.998)
            return 2.5;
        else
            return 4.0;
    }

    bool ORBmatcher::CheckDistEpipolarLine(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &F12,
                                           const KeyFrame *pKF2) {
        // Epipolar line in second image l = x1'F12 = [a b c]
        const float a = kp1.pt.x * F12.at<float>(0, 0) + kp1.pt.y * F12.at<float>(1, 0) + F12.at<float>(2, 0);
        const float b = kp1.pt.x * F12.at<float>(0, 1) + kp1.pt.y * F12.at<float>(1, 1) + F12.at<float>(2, 1);
        const float c = kp1.pt.x * F12.at<float>(0, 2) + kp1.pt.y * F12.at<float>(1, 2) + F12.at<float>(2, 2);

        const float num = a * kp2.pt.x + b * kp2.pt.y + c;

        const float den = a * a + b * b;

        if (den == 0)
            return false;

        const float dsqr = num * num / den;

        return dsqr < 3.84 * pKF2->mvLevelSigma2[kp2.octave];
    }


    // Search by kd tree functions
    // Match all key-points in the frame to all map-points in the surrounding of key-frame pKF within a SEARCH_RADIUS * search_radius_factor
    int ORBmatcher::SearchByKDTree (KeyFrame *pKF, Frame &F, Map* mpMap, vector<MapPoint *> &vpMapPointMatches)
    {
        const float search_radius_factor = 20.0;
        unsigned int numberOfMatches = 0;
        const int numberOfFeatures = F.N;
        const bool isDebugMode = false;
        vpMapPointMatches = vector<MapPoint *>(F.N, static_cast<MapPoint *>(NULL));
        // Probe the KD tree for all neighboring map-points within a SEARCH_RADIUS
        float keyframePose [NUMBER_OF_AXES] = {
                                                pKF->GetPoseInverse().at<float>(0,3),
                                                pKF->GetPoseInverse().at<float>(1,3),
                                                pKF->GetPoseInverse().at<float>(2,3)
                                                };

        vector<MapPoint *> neighbors = mpMap->GetKDTreeNeighbors(keyframePose, SEARCH_RADIUS * search_radius_factor);

//        cout << "Number of neighbors = " << neighbors.size() << endl;
//        cout << "Number of features = " << numberOfFeatures << endl;

        // iterate through all the key-points in the current frame
        for ( unsigned int featureCounter = 0; featureCounter < numberOfFeatures; featureCounter++ ) {
            // validate if the current key-point has a positive depth i.e., is a valid key-point
            if (F.mvDepth[featureCounter] > 0) {

                    // Descriptor distance variables
                    int bestDist1 = 256;
                    int bestIdxF = -1;
                    int bestDist2 = 256;

                    const int NumberOfNeigbors = neighbors.size();

                    // iterate through all the neighboring map-points in the map
                    for (int neighborCounter = 0; neighborCounter < NumberOfNeigbors; neighborCounter++)
                    {
                        MapPoint *pMP = neighbors.at(neighborCounter);

                        // skip the map-point if it is NULL, Bad, or from a previous session and has already been marked
                        if (pMP == NULL)
                            continue;
                        else if (pMP->isBad())
                            continue;

                        // Get the descriptor distance between the key-point and its map-point neighbor
                        const int dist = DescriptorDistance(F.mDescriptors.row(featureCounter), pMP->GetDescriptor());

                        // keep track of the best and second best match
                        if (dist < bestDist1) {
                            bestDist2 = bestDist1;
                            bestDist1 = dist;
                            bestIdxF = neighborCounter;
                        } else if (dist < bestDist2) {
                            bestDist2 = dist;
                        }
                    }

                    if (bestDist1 <= TH_LOW) // if we have a match within the pre-defined TH_LOW
                    {
                        // retrieve the best-matching map-point
                        MapPoint *pMP = neighbors.at(bestIdxF);
                        // make sure it passes the ratio test for feature-matching
                        if (static_cast<float>(bestDist1) < mfNNratio * static_cast<float>(bestDist2)) {
                            vpMapPointMatches [featureCounter] = pMP;
                            // mark the map-point as marked
                            numberOfMatches++;
                            if (pMP->GetFromPreviousSession() == true)
                                pMP->IncrementNumberOfOccurencesInNewSession();
//                                numberOfMatches++;
                        }
                    }
                }
            }

//        if (isDebugMode == true)
//            cout << "Search radius = " << SEARCH_RADIUS * search_radius_factor << endl;
//        cout << "Number of matches from inside the function = " << numberOfMatches << endl;
        return numberOfMatches;

        }

    // Feature match all key-points in the frame F by finding their 3D neighbors in the Map mp
    int ORBmatcher::SearchByKDTree(Frame &F, Map *mpMap)
    {
        unsigned int numberOfMatches = 0;
        const int numberOfFeatures = F.N;


        // iterate through all the key-points in the current frame
        for ( unsigned int featureCounter = 0; featureCounter < numberOfFeatures; featureCounter++ ) {

            if (F.mvpMapPoints[featureCounter] != NULL)
            {
                continue;
            }
            // validate if the current key-point has a positive depth i.e., is a valid key-point
            if (F.mvDepth[featureCounter] > 0) {
                // Get the 3D position of the current key-point w.r.t to the map
                cv::Mat featurePoint = F.UnprojectStereo(featureCounter);

                // continue only if we were able to accurately position it in the 3D map
                if ((featurePoint.empty() == false)) {
                    // Store the 3D position of the key-point in a float array
                    float coordinates[NUMBER_OF_AXES] = {
                            featurePoint.at<float>(X_AXIS, 0),
                            featurePoint.at<float>(Y_AXIS, 0),
                            featurePoint.at<float>(Z_AXIS, 0)
                    };

                    // Probe the KD tree for all neighboring map-points within a SEARCH_RADIUS
                    vector<MapPoint *> neighbors = mpMap->GetKDTreeNeighbors(coordinates, SEARCH_RADIUS);

                    // Descriptor distance variables
                    int bestDist1 = 256;
                    int bestIdxF = -1;
                    int bestDist2 = 256;

                    const int NumberOfNeigbors = neighbors.size();

                    // iterate through all the neighboring map-points in the map
                    for (int neighborCounter = 0; neighborCounter < NumberOfNeigbors; neighborCounter++) {
                        MapPoint *pMP = neighbors.at(neighborCounter);

                        // skip the map-point if it is NULL, Bad, or from a previous session and has already been marked
                        if (pMP == NULL)
                            continue;
                        else if (pMP->isBad())
                            continue;
//                        else if ((pMP->GetFromPreviousSession() == true) && (pMP->GetNumberOfOccurencesInNewSession() > 0))
//                            continue;

                        // Get the descriptor distance between the key-point and its map-point neighbor
                        const int dist = DescriptorDistance(F.mDescriptors.row(featureCounter), pMP->GetDescriptor());

                        // keep track of the best and second best match
                        if (dist < bestDist1) {
                            bestDist2 = bestDist1;
                            bestDist1 = dist;
                            bestIdxF = neighborCounter;
                        } else if (dist < bestDist2) {
                            bestDist2 = dist;
                        }
                    }

                    if (bestDist1 <= TH_LOW) // if we have a match within the pre-defined TH_LOW
                    {
                        // retrieve the best-matching map-point
                        MapPoint *pMP = neighbors.at(bestIdxF);
                        // make sure it passes the ratio test for feature-matching
                        if (static_cast<float>(bestDist1) < mfNNratio * static_cast<float>(bestDist2))
                        {
                                numberOfMatches++;
                                F.mvpMapPoints[featureCounter] = pMP;
                                // mark the map-point as marked
                                if (pMP->GetFromPreviousSession() == true)
                                    pMP->IncrementNumberOfOccurencesInNewSession();
                        }
                    }
                }
            }
        }
//        cout << "Number of new matches = " << numberOfMatches << endl;
        return numberOfMatches;
    }


    int ORBmatcher::SearchByKDTree(KeyFrame *pKF, Map *mpMap, vector<MapPoint *> &vpMapPointMatches)
    {

        const int NUMBER_OF_AXES = 3;
        unsigned int numberOfMatches = 0;
        const int numberOfFeatures = pKF->GetIndexedMapPoints().size();

        vpMapPointMatches = vector<MapPoint *>(numberOfFeatures, static_cast<MapPoint *>(NULL));

        vector <MapPoint*> indexedMps = pKF->GetIndexedMapPoints();
        cout << "Number of features = " << numberOfFeatures << endl;
        // iterate through all the key-points in the current frame
        for ( unsigned int featureCounter = 0; featureCounter < numberOfFeatures; featureCounter++ ) {


            if (indexedMps.at(featureCounter) == NULL)
            {
                continue;
            }

            MapPoint *potentialMP = indexedMps.at(featureCounter);



            if (potentialMP == NULL)
                continue;

            std::map<KeyFrame*, MapPoint_Observation> allObs = potentialMP->GetMpObservations();

            MapPoint_Observation obs;

            if (allObs.find(pKF) ==  allObs.end())
                continue;

            else
                obs = allObs.find(pKF)->second;
//                continue;
//            obs = potentialMP->GetMpObservations().find(pKF)->second;
            float x = obs.Keypoint.pt.x;
            float y = obs.Keypoint.pt.y;
            float z = obs.depth;

//            cout << "(x, y, z) = " << x << ", " << y << ", " << z << endl;

            {
                // Get the 3D position of the current key-point w.r.t to the map
                cv::Mat featurePoint = pKF->UnprojectStereo(x, y , z);
//                cout << featurePoint << endl;

                // continue only if we were able to accurately position it in the 3D map
                if ((featurePoint.empty() == false)) {
                    // Store the 3D position of the key-point in a float array
                    float coordinates[NUMBER_OF_AXES] = {
                            featurePoint.at<float>(X_AXIS, 0),
                            featurePoint.at<float>(Y_AXIS, 0),
                            featurePoint.at<float>(Z_AXIS, 0)
                    };

                    // Probe the KD tree for all neighboring map-points within a SEARCH_RADIUS
                    vector<MapPoint *> neighbors = mpMap->GetKDTreeNeighbors(coordinates, SEARCH_RADIUS * 10);

                    // Descriptor distance variables
                    int bestDist1 = 256;
                    int bestIdxF = -1;
                    int bestDist2 = 256;

                    const int NumberOfNeigbors = neighbors.size();

                    // iterate through all the neighboring map-points in the map
                    for (int neighborCounter = 0; neighborCounter < NumberOfNeigbors; neighborCounter++) {
                        MapPoint *pMP = neighbors.at(neighborCounter);

                        // skip the map-point if it is NULL, Bad, or from a previous session and has already been marked
                        if (pMP == NULL)
                            continue;
                        else if (pMP->isBad())
                            continue;
//                        else if ((pMP->GetFromPreviousSession() == true) && (pMP->GetNumberOfOccurencesInNewSession() > 0))
//                            continue;

                        // Get the descriptor distance between the key-point and its map-point neighbor
                        const int dist = DescriptorDistance(potentialMP->GetDescriptor(), pMP->GetDescriptor());

                        // keep track of the best and second best match
                        if (dist < bestDist1) {
                            bestDist2 = bestDist1;
                            bestDist1 = dist;
                            bestIdxF = neighborCounter;
                        } else if (dist < bestDist2) {
                            bestDist2 = dist;
                        }
                    }

                    if (bestDist1 <= TH_LOW) // if we have a match within the pre-defined TH_LOW
                    {
                        // retrieve the best-matching map-point
                        MapPoint *pMP = neighbors.at(bestIdxF);
                        // make sure it passes the ratio test for feature-matching
                        if (static_cast<float>(bestDist1) < mfNNratio * static_cast<float>(bestDist2))
                        {
                            numberOfMatches++;
                            vpMapPointMatches [featureCounter] = pMP;
                            // mark the map-point as marked
                            if (pMP->GetFromPreviousSession() == true)
                                pMP->IncrementNumberOfOccurencesInNewSession();
                        }
                    }
                }
            }
        }
        cout << "Number of new matches = " << numberOfMatches << endl;
        return numberOfMatches;
    }



    // Feature match all key-point in the frame F by searching for their 3D neighbors in the Map mp and mark them in vpMapPointMatches
    int ORBmatcher::SearchByKDTree(Frame &F, Map *mpMap, vector <MapPoint*> &vpMapPointMatches, float search_radius_multiplier)
    {
        unsigned int numberOfMatches = 0;
        const int numberOfFeatures = F.N;
        vpMapPointMatches = vector<MapPoint *>(numberOfFeatures, static_cast<MapPoint *>(NULL));

        // iterate through all the key-points in the current frame
        for ( unsigned int featureCounter = 0; featureCounter < numberOfFeatures; featureCounter++ ) {
            // validate if the current key-point has a positive depth i.e., is a valid key-point
            if (F.mvDepth[featureCounter] > 0) {
                // Get the 3D position of the current key-point w.r.t to the map
                cv::Mat featurePoint = F.UnprojectStereo(featureCounter);

                // continue only if we were able to accurately position it in the 3D map
                if ((featurePoint.empty() == false)) {
                    // Store the 3D position of the key-point in a float array
                    float coordinates[NUMBER_OF_AXES] = {
                            featurePoint.at<float>(X_AXIS, 0),
                            featurePoint.at<float>(Y_AXIS, 0),
                            featurePoint.at<float>(Z_AXIS, 0)
                    };

                    // Probe the KD tree for all neighboring map-points within a SEARCH_RADIUS
                    vector<MapPoint *> neighbors = mpMap->GetKDTreeNeighbors(coordinates, SEARCH_RADIUS * search_radius_multiplier);

                    // Descriptor distance variables
                    int bestDist1 = 256;
                    int bestIdxF = -1;
                    int bestDist2 = 256;

                    const int NumberOfNeigbors = neighbors.size();

                    // iterate through all the neighboring map-points in the map
                    for (int neighborCounter = 0; neighborCounter < NumberOfNeigbors; neighborCounter++) {
                        MapPoint *pMP = neighbors.at(neighborCounter);

                        // skip the map-point if it is NULL, Bad, or from a previous session and has already been marked
                        if (pMP == NULL)
                            continue;
                        else if (pMP->isBad())
                            continue;

                        // Get the descriptor distance between the key-point and its map-point neighbor
                        const int dist = DescriptorDistance(F.mDescriptors.row(featureCounter), pMP->GetDescriptor());
                        // keep track of the best and second best match
                        if (dist < bestDist1) {
                            bestDist2 = bestDist1;
                            bestDist1 = dist;
                            bestIdxF = neighborCounter;
                        } else if (dist < bestDist2) {
                            bestDist2 = dist;
                        }
                    }

                    if (bestDist1 <= TH_LOW) // if we have a match within the pre-defined TH_LOW
                    {
                        // retrieve the best-matching map-point
                        MapPoint *pMP = neighbors.at(bestIdxF);
                        // make sure it passes the ratio test for feature-matching
                        if (static_cast<float>(bestDist1) <= mfNNratio * static_cast<float>(bestDist2))
                        {
                            numberOfMatches++; // increment the number of matches
                            vpMapPointMatches [featureCounter] = pMP; // mark the corresponding map-point we have matched to
                            if (pMP->GetFromPreviousSession() == true)                             // mark the map-point as marked
                                pMP->IncrementNumberOfOccurencesInNewSession();
                        }
                    }
                }
            }
        }
        return numberOfMatches;
    }


    // Unfinished work
    int ORBmatcher::SearchByLabels (KeyFrame *pKF, Frame &F, std::vector<MapPoint*> &vpMapPointMatches)
    {

//        static unsigned int counter
        const bool isDebugMode = false;
        unsigned int nmatches = 0;

        vpMapPointMatches = vector<MapPoint *>(F.N, static_cast<MapPoint *>(NULL));

        if (isDebugMode)
            cout << "Diff_Operations - SearchByLabels()" << endl;

        if (pKF->LoadedFromDisk() == false) {
            if (isDebugMode)
                cout
                        << "Diff_Operation: SearchByLabels() - exitting early because keyframe is from current session"
                        << endl;

            return -1;
        }

        vector<MapPoint *> kfMapPoints = pKF->GetIndexedMapPoints();


        // go through all the key-points in the current frame
        const int numberOfKeypoints = F.N;
        const int numberOfMapPoints = pKF->GetIndexedMapPoints().size();

        // iterate through the whole keyframe feature/map-point list
        for (int kfFeatureCounter = 0; kfFeatureCounter < numberOfMapPoints; kfFeatureCounter++)
        {
            MapPoint* pMP = kfMapPoints.at(kfFeatureCounter);

//            cout << "pMP = " << pMP->mnId << " : ";

            if (pMP == NULL)
                continue;
            else if (pMP->isBad())
                continue;
            if ( (pMP->GetFromPreviousSession() == true) && (pMP->GetNumberOfOccurencesInNewSession() > 0))
                continue;


            unsigned char pMPLabel = pMP->GetSemanticLabel();


            int bestDist1 = 256;
            int bestIdxF = -1;
            int bestDist2 = 256;


            for (int frameFeatureCounter = 0; frameFeatureCounter < numberOfKeypoints; frameFeatureCounter++)
            {
                if (vpMapPointMatches[frameFeatureCounter] != NULL)
                    continue;

                unsigned char kpLabel = F.mvLabels.at(frameFeatureCounter);

                if (kpLabel != pMPLabel)
                    continue;


                // Get the descriptor of the current key-point
                const cv::Mat &dPF = F.mDescriptors.row(frameFeatureCounter);
                const cv::Mat &dKF = pMP->GetBestDescriptor(pKF);




                const int dist = DescriptorDistance(dKF, dPF);

                if (dist < bestDist1) {
                    bestDist2 = bestDist1;
                    bestDist1 = dist;
                    bestIdxF = frameFeatureCounter;
                } else if (dist < bestDist2) {
                    bestDist2 = dist;
                }
            }

            if (bestDist1 <= TH_LOW)
            {
                if (static_cast<float>(bestDist1) < mfNNratio * static_cast<float>(bestDist2))
                {
                    vpMapPointMatches[bestIdxF] = pMP;
                    const cv::KeyPoint &kp = pMP->mpObservation.find(pKF)->second.Keypoint;//mpObservation
                    nmatches++;
                    // Diff operation
                    if ( pMP->GetFromPreviousSession() == true )
                    {
                        pMP->IncrementNumberOfOccurencesInNewSession();
                    }

//                    cout << "matched with distance = " << bestDist1 << endl;
                }
                else;
//                    cout << "no match because of ratio test and distance = " << bestDist1 << endl;
            }
            else;
//                cout << "no match and distance = " << bestDist1 << endl;

        }

//        cout << "The number of map-points are " << pKF->GetIndexedMapPoints().size() << endl;
//        cout << "The number of key-points are " << F.N << endl;
//        cout << "The number of matches are " << nmatches << endl;
        return nmatches;


    }

    // Brute force matching all map-points in the keyframe pKF with all the map-point in pF
    int ORBmatcher::BruteForceMatchingWithKeyframe(KeyFrame *pKF, Frame *pF, std::vector<MapPoint *> &vpMapPointMatches,
                                                   float threshold) {
        const bool isDebugMode = false;
        unsigned int nmatches = 0;
        vpMapPointMatches = vector<MapPoint *>(pF->N, static_cast<MapPoint *>(NULL));
        if (isDebugMode)
            cout << "Diff_Operations - BruteForceMatchingWithKeyframe()" << endl;
        if (pKF->LoadedFromDisk() == false) {
            if (isDebugMode)
                cout
                        << "Diff_Operation: BruteForceMatchingWithKeyFram() - exitting early because keyframe is from current session"
                        << endl;

            return -1;
        }

        vector<MapPoint *> kfMapPoints = pKF->GetIndexedMapPoints();
        // go through all the key-points in the current frame
        const int numberOfKeypoints = pF->N;
        const int numberOfMapPoints = pKF->GetIndexedMapPoints().size();
        // iterate through the whole keyframe feature/map-point list
        for (int kfFeatureCounter = 0; kfFeatureCounter < numberOfMapPoints; kfFeatureCounter++)
        {
            MapPoint* pMP = kfMapPoints.at(kfFeatureCounter);
            if (pMP == NULL)
                continue;
            else if (pMP->isBad())
                continue;
            if ( (pMP->GetFromPreviousSession() == true) && (pMP->GetNumberOfOccurencesInNewSession() > 0))
                continue;
            int bestDist1 = 256;
            int bestIdxF = -1;
            int bestDist2 = 256;
            for (int frameFeatureCounter = 0; frameFeatureCounter < numberOfKeypoints; frameFeatureCounter++)
            {
                if (vpMapPointMatches[frameFeatureCounter] != NULL)
                    continue;
                // Get the descriptor of the current key-point
                const cv::Mat &dPF = pF->mDescriptors.row(frameFeatureCounter);
                const cv::Mat &dKF = pMP->GetBestDescriptor(pKF);
                const int dist = DescriptorDistance(dKF, dPF);
                    if (dist < bestDist1) {
                        bestDist2 = bestDist1;
                        bestDist1 = dist;
                        bestIdxF = frameFeatureCounter;
                    } else if (dist < bestDist2) {
                        bestDist2 = dist;
                    }
            }
            if (bestDist1 <= TH_LOW)
            {
                if (static_cast<float>(bestDist1) < mfNNratio * static_cast<float>(bestDist2))
                {
                        vpMapPointMatches[bestIdxF] = pMP;
                        const cv::KeyPoint &kp = pMP->mpObservation.find(pKF)->second.Keypoint;//mpObservation
                        nmatches++;
                        // Diff operation
                        if ( pMP->GetFromPreviousSession() == true )
                        {
                            pMP->IncrementNumberOfOccurencesInNewSession();
                        }
                }
                else;
            }
            else;
        }
        return nmatches;
    }

    // Feature match map-points from pKF with the frame F using their BoW vectors
    int ORBmatcher::SearchByBoW(KeyFrame *pKF, Frame &F, vector<MapPoint *> &vpMapPointMatches)
    {
        //get all the map point matches for the current keyframe
        bool isDebugMode = true;
        vector <MapPoint *> kfMapPoints;
        if (pKF->LoadedFromDisk() == false)//if it is from the current session, get map point matches using key point indexing
            kfMapPoints = pKF->GetMapPointMatches();
        else
            kfMapPoints = pKF->GetIndexedMapPoints();//else, only get the indexed map point matches

        const vector<MapPoint *> vpMapPointsKF = kfMapPoints;//indexed map points

        //Map point matches vector between current frame and the reference keyframe
        vpMapPointMatches = vector<MapPoint *>(F.N, static_cast<MapPoint *>(NULL));

        //mFeatVect for the keyframe
        const DBoW2::FeatureVector &vFeatVecKF = pKF->mFeatVec;

        //Number of map point matches between current frame and keyframe
        int nmatches = 0;
        int prevSessionMatches = 0; // Number of matches between the current frame and the map-points from the previous session


        vector<int> rotHist[HISTO_LENGTH];
        for (int i = 0; i < HISTO_LENGTH; i++)
            rotHist[i].reserve(500);
        const float factor = 1.0f / HISTO_LENGTH;

        // We perform the matching over ORB that belong to the same vocabulary node (at a certain level)
        DBoW2::FeatureVector::const_iterator KFit = vFeatVecKF.begin();
        DBoW2::FeatureVector::const_iterator Fit = F.mFeatVec.begin();
        DBoW2::FeatureVector::const_iterator KFend = vFeatVecKF.end();
        DBoW2::FeatureVector::const_iterator Fend = F.mFeatVec.end();


        while (KFit != KFend && Fit != Fend) {
            if (KFit->first == Fit->first) {
                //The main problem lies here
                //I have to fix up their indicies here
                const vector<unsigned int> vIndicesKF = KFit->second;
                const vector<unsigned int> vIndicesF = Fit->second;

                for (size_t iKF = 0; iKF < vIndicesKF.size(); iKF++) {
                    const unsigned int realIdxKF = vIndicesKF[iKF];

                    MapPoint *pMP = vpMapPointsKF[realIdxKF];
                    if (!pMP)
                        continue;
                    if (pMP->isBad())
                        continue;
                    const cv::Mat &dKF = pMP->GetBestDescriptor(pKF);//Get distinctive descriptor of current map point
                    int bestDist1 = 256;
                    int bestIdxF = -1;
                    int bestDist2 = 256;
                    for (size_t iF = 0; iF < vIndicesF.size(); iF++) {
                        const unsigned int realIdxF = vIndicesF[iF];

                        if (vpMapPointMatches[realIdxF])
                            continue;
                        const cv::Mat &dF = F.mDescriptors.row(realIdxF);
                        const int dist = DescriptorDistance(dKF, dF);
                        if (dist < bestDist1) {
                            bestDist2 = bestDist1;
                            bestDist1 = dist;
                            bestIdxF = realIdxF;
                        } else if (dist < bestDist2) {
                            bestDist2 = dist;
                        }
                    }

                    if (bestDist1 <= TH_LOW) {
                        if (static_cast<float>(bestDist1) < mfNNratio * static_cast<float>(bestDist2)) {
                            vpMapPointMatches[bestIdxF] = pMP;
                            const cv::KeyPoint &kp = pMP->mpObservation.find(pKF)->second.Keypoint;//mpObservation

//                            cout << kp.pt.x << ", " << kp.pt.y << endl;

                            if (mbCheckOrientation)
                            {
                                float rot = kp.angle - F.mvKeys[bestIdxF].angle;
                                if (rot < 0.0)
                                    rot += 360.0f;
                                int bin = round(rot * factor);
                                if (bin == HISTO_LENGTH)
                                    bin = 0;
                                assert(bin >= 0 && bin < HISTO_LENGTH);
                                rotHist[bin].push_back(bestIdxF);
                            }

                            if (majorityVoting == true)
                                pMP->AppendLabel(F.mvLabels[bestIdxF]);


                            nmatches++;
                            // Diff operation
                            if (pMP->GetFromPreviousSession() == true)
                            {
                                pMP->IncrementNumberOfOccurencesInNewSession();
                                prevSessionMatches++;
                            }
                        }
                    }
                }
                KFit++;
                Fit++;
            } else if (KFit->first < Fit->first) {
                KFit = vFeatVecKF.lower_bound(Fit->first);
            } else {
                Fit = F.mFeatVec.lower_bound(KFit->first);
            }
        }

        if (mbCheckOrientation) {
            int ind1 = -1;
            int ind2 = -1;
            int ind3 = -1;

            ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

            for (int i = 0; i < HISTO_LENGTH; i++) {
                if (i == ind1 || i == ind2 || i == ind3)
                    continue;
                for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
                    vpMapPointMatches[rotHist[i][j]] = static_cast<MapPoint *>(NULL);
                    nmatches--;
                }
            }
        }
        return nmatches;
    }

    int ORBmatcher::SearchByProjection(KeyFrame *pKF, cv::Mat Scw, const vector<MapPoint *> &vpPoints,
                                       vector<MapPoint *> &vpMatched, int th)
    {
        // Get Calibration Parameters for later projection
        const float &fx = pKF->fx;
        const float &fy = pKF->fy;
        const float &cx = pKF->cx;
        const float &cy = pKF->cy;

        unsigned int prevSessionMatches = 0;

        // Decompose Scw
        cv::Mat sRcw = Scw.rowRange(0, 3).colRange(0, 3);
        const float scw = sqrt(sRcw.row(0).dot(sRcw.row(0)));
        cv::Mat Rcw = sRcw / scw;
        cv::Mat tcw = Scw.rowRange(0, 3).col(3) / scw;
        cv::Mat Ow = -Rcw.t() * tcw;

        // Set of MapPoints already found in the KeyFrame
        set<MapPoint *> spAlreadyFound(vpMatched.begin(), vpMatched.end());
        spAlreadyFound.erase(static_cast<MapPoint *>(NULL));

        int nmatches = 0;

        // For each Candidate MapPoint Project and Match
        for (int iMP = 0, iendMP = vpPoints.size(); iMP < iendMP; iMP++) {
            MapPoint *pMP = vpPoints[iMP];

            // Discard Bad MapPoints and already found
            if (pMP->isBad() || spAlreadyFound.count(pMP))
                continue;

            // Get 3D Coords.
            cv::Mat p3Dw = pMP->GetWorldPos();

            // Transform into Camera Coords.
            cv::Mat p3Dc = Rcw * p3Dw + tcw;

            // Depth must be positive
            if (p3Dc.at<float>(2) < 0.0)
                continue;

            // Project into Image
            const float invz = 1 / p3Dc.at<float>(2);
            const float x = p3Dc.at<float>(0) * invz;
            const float y = p3Dc.at<float>(1) * invz;

            const float u = fx * x + cx;
            const float v = fy * y + cy;

            // Point must be inside the image
            if (!pKF->IsInImage(u, v))
                continue;

            // Depth must be inside the scale invariance region of the point
            const float maxDistance = pMP->GetMaxDistanceInvariance();
            const float minDistance = pMP->GetMinDistanceInvariance();
            cv::Mat PO = p3Dw - Ow;
            const float dist = cv::norm(PO);

            if (dist < minDistance || dist > maxDistance)
                continue;

            // Viewing angle must be less than 60 deg
            cv::Mat Pn = pMP->GetNormal();

            if (PO.dot(Pn) < 0.5 * dist)
                continue;

            int nPredictedLevel = pMP->PredictScale(dist, pKF);

            // Search in a radius
            const float radius = th * pKF->mvScaleFactors[nPredictedLevel];

            if (pKF->LoadedFromDisk())
                cerr << "Problem here" << endl;

            const vector<size_t> vIndices = pKF->GetFeaturesInArea(u, v, radius);

            if (vIndices.empty())
                continue;

            // Match to the most similar keypoint in the radius
            const cv::Mat dMP = pMP->GetDescriptor();

            int bestDist = 256;
            int bestIdx = -1;
            for (vector<size_t>::const_iterator vit = vIndices.begin(), vend = vIndices.end(); vit != vend; vit++) {
                const size_t idx = *vit;
                if (vpMatched[idx])
                    continue;

                const int &kpLevel = pKF->mvKeysUn[idx].octave;


                if (kpLevel < nPredictedLevel - 1 || kpLevel > nPredictedLevel)
                    continue;

                const cv::Mat &dKF = pKF->mDescriptors.row(idx);

                const int dist = DescriptorDistance(dMP, dKF);

                if (dist < bestDist) {
                    bestDist = dist;
                    bestIdx = idx;
                }
            }

            if (bestDist <= TH_LOW) {
                vpMatched[bestIdx] = pMP;
                nmatches++;



                if (pMP->GetFromPreviousSession() == true)
                {
                        pMP->IncrementNumberOfOccurencesInNewSession();
                        prevSessionMatches++;
                }
            }
        }
        return nmatches;
    }

    int ORBmatcher::SearchForInitialization(Frame &F1, Frame &F2, vector<cv::Point2f> &vbPrevMatched,
                                            vector<int> &vnMatches12, int windowSize) {
        int nmatches = 0;
        vnMatches12 = vector<int>(F1.mvKeysUn.size(), -1);

        vector<int> rotHist[HISTO_LENGTH];
        for (int i = 0; i < HISTO_LENGTH; i++)
            rotHist[i].reserve(500);
        const float factor = 1.0f / HISTO_LENGTH;

        vector<int> vMatchedDistance(F2.mvKeysUn.size(), INT_MAX);
        vector<int> vnMatches21(F2.mvKeysUn.size(), -1);

        for (size_t i1 = 0, iend1 = F1.mvKeysUn.size(); i1 < iend1; i1++) {
            cv::KeyPoint kp1 = F1.mvKeysUn[i1];
            int level1 = kp1.octave;
            if (level1 > 0)
                continue;

            vector<size_t> vIndices2 = F2.GetFeaturesInArea(vbPrevMatched[i1].x, vbPrevMatched[i1].y, windowSize,
                                                            level1, level1);

            if (vIndices2.empty())
                continue;

            cv::Mat d1 = F1.mDescriptors.row(i1);

            int bestDist = INT_MAX;
            int bestDist2 = INT_MAX;
            int bestIdx2 = -1;

            for (vector<size_t>::iterator vit = vIndices2.begin(); vit != vIndices2.end(); vit++) {
                size_t i2 = *vit;

                cv::Mat d2 = F2.mDescriptors.row(i2);

                int dist = DescriptorDistance(d1, d2);

                if (vMatchedDistance[i2] <= dist)
                    continue;

                if (dist < bestDist) {
                    bestDist2 = bestDist;
                    bestDist = dist;
                    bestIdx2 = i2;
                } else if (dist < bestDist2) {
                    bestDist2 = dist;
                }
            }

            if (bestDist <= TH_LOW) {
                if (bestDist < (float) bestDist2 * mfNNratio) {
                    if (vnMatches21[bestIdx2] >= 0) {
                        vnMatches12[vnMatches21[bestIdx2]] = -1;
                        nmatches--;
                    }
                    vnMatches12[i1] = bestIdx2;
                    vnMatches21[bestIdx2] = i1;
                    vMatchedDistance[bestIdx2] = bestDist;
                    nmatches++;

                    if (mbCheckOrientation) {
                        float rot = F1.mvKeysUn[i1].angle - F2.mvKeysUn[bestIdx2].angle;
                        if (rot < 0.0)
                            rot += 360.0f;
                        int bin = round(rot * factor);
                        if (bin == HISTO_LENGTH)
                            bin = 0;
                        assert(bin >= 0 && bin < HISTO_LENGTH);
                        rotHist[bin].push_back(i1);
                    }
                }
            }

        }

        if (mbCheckOrientation) {
            int ind1 = -1;
            int ind2 = -1;
            int ind3 = -1;

            ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

            for (int i = 0; i < HISTO_LENGTH; i++) {
                if (i == ind1 || i == ind2 || i == ind3)
                    continue;
                for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
                    int idx1 = rotHist[i][j];
                    if (vnMatches12[idx1] >= 0) {
                        vnMatches12[idx1] = -1;
                        nmatches--;
                    }
                }
            }

        }

        //Update prev matched
        for (size_t i1 = 0, iend1 = vnMatches12.size(); i1 < iend1; i1++)
            if (vnMatches12[i1] >= 0)
                vbPrevMatched[i1] = F2.mvKeysUn[vnMatches12[i1]].pt;

        return nmatches;
    }

    int ORBmatcher::SearchByBoWForMapPoints(KeyFrame *rc, KeyFrame *lk, vector<MapPoint *> &vpMatches)
    {
        //Debugging
        const bool debug_mode = false;
        const bool isWarningMessages = false;
        int numberOfBoWMatches = 0;
        // Get all map points for the relocalization candidate rc
        vector <MapPoint *> rcMapPoints;
        rcMapPoints = rc->GetIndexedMapPoints();
        if (debug_mode)
            cout << "Number of map points in relocalization candidate = " << rcMapPoints.size() << endl;
        // Get all map points from the lost keyframe or keyframe to relocalize in the map
        vector <MapPoint *> lkMapPoints;
        lkMapPoints = lk->GetIndexedMapPoints();
        if (debug_mode)
            cout << "Number of map points in lost keyframe = " << lkMapPoints.size() << endl;


        // Map point matches vector that relate lost keyframe (lk) to relocalization candidate (rc)
        // Maps map point indicies from lost keyframes to relocalization candidate
        vpMatches = vector<MapPoint *>(lkMapPoints.size(), static_cast<MapPoint *>(NULL));

        // Feature vectors for both keyframes
        const DBoW2::FeatureVector &vFeatVecRC = rc->mFeatVec;
        const DBoW2::FeatureVector &vFeatVecLK = lk->mFeatVec;

        //Number of map point matches between current frame and keyframe
        int nmatches = 0;

        vector<int> rotHist[HISTO_LENGTH];
        for (int i = 0; i < HISTO_LENGTH; i++)
            rotHist[i].reserve(500);
        const float factor = 1.0f / HISTO_LENGTH;

        // We perform the matching over ORB that belong to the same vocabulary node (at a certain level)
        DBoW2::FeatureVector::const_iterator KFit = vFeatVecRC.begin();
        DBoW2::FeatureVector::const_iterator Fit = vFeatVecLK.begin();
        DBoW2::FeatureVector::const_iterator KFend = vFeatVecRC.end();
        DBoW2::FeatureVector::const_iterator Fend = vFeatVecLK.end();

        // Go through all the feature vector elements in both KFit and Fit
        while (KFit != KFend && Fit != Fend) {
            // Initialization
            if (KFit->first == Fit->first) {
                // Get the start indicies
                const vector<unsigned int> vIndicesKF = KFit->second;
                const vector<unsigned int> vIndicesF = Fit->second;

                if (debug_mode) {
                    cout << "Number of elements in KF = " << vIndicesKF.size() << endl;
                    cout << "Number of elements in LK = " << vIndicesF.size() << endl;
                }

                // Outer loop : Iterate through all elements in the relocalization candidate for the current vocabulary level
                for (size_t iKF = 0; iKF < vIndicesKF.size(); iKF++) {

                    // Get the index of the map-point
                    const unsigned int realIdxKF = vIndicesKF[iKF];
                    MapPoint *pMP = rcMapPoints[realIdxKF];

                    if (debug_mode)
                        cout << endl << "RC mp [" << realIdxKF << "] - matches = ";

                    if (!pMP)
                        continue;

                    if (pMP->isBad())
                        continue;

                    // Get the descriptor for the map-point
                    const cv::Mat &dKF = pMP->GetDescriptor();

                    int bestDist1 = 256;
                    int bestIdxF = -1;
                    int bestDist2 = 256;

                    // Now iterate through all map-point in the feature vector of the lost keyframe at the current vocabulary level
                    for (size_t iF = 0; iF < vIndicesF.size(); iF++) {

                        // Get the index of the current map point
                        const unsigned int realIdxF = vIndicesF[iF];
                        // is the map point already matched? If yes, then continue
                        if (vpMatches[realIdxF])
                            continue;

                        MapPoint *lkpMP = lkMapPoints[realIdxF];
                        if (debug_mode)
                            cout  << realIdxF << "";

                        const cv::Mat &dF = lkpMP->GetDescriptor();
                        const int dist = DescriptorDistance(dKF, dF);

                        if (dist < bestDist1) {
                            bestDist2 = bestDist1;
                            bestDist1 = dist;
                            bestIdxF = realIdxF;
                        } else if (dist < bestDist2) {
                            bestDist2 = dist;
                        }
                    }

                    if (bestDist1 <= TH_LOW) {

                        if (debug_mode)
                            cout << endl << "Best distance = " << bestDist1 << " for id = " << bestIdxF << endl;

                        if (static_cast<float>(bestDist1) < mfNNratio * static_cast<float>(bestDist2))
                        {
                            vpMatches[bestIdxF] = pMP;

                            if (pMP->GetFromPreviousSession() == true)
                            {
                                pMP->IncrementNumberOfOccurencesInNewSession();
                                numberOfBoWMatches++;
                            }

                            const cv::KeyPoint &kp = pMP->mpObservation.find(rc)->second.Keypoint;//mpObservation
                            const cv::KeyPoint &lkKp = lkMapPoints[bestIdxF]->mpObservation.find(lk)->second.Keypoint;

                            if (mbCheckOrientation)
                            {
                                if (kp.angle > 1000) {
                                    if (isWarningMessages)
                                        cerr << "Erraneous value" << endl;
                                    continue;
                                }

                                float rot = kp.angle - lkKp.angle;

                                if (rot < 0.0)
                                    rot += 360.0f;
                                int bin = round(rot * factor);
                                if (bin == HISTO_LENGTH)
                                    bin = 0;
                                if (!(bin >= 0 && bin < HISTO_LENGTH))
                                {
                                    cerr << "Error here" << endl;
                                    continue;
                                }
                                rotHist[bin].push_back(bestIdxF);
                            }
                            nmatches++;
                        }
                    }

                }
                KFit++;
                Fit++;
            } else if (KFit->first < Fit->first) {
                KFit = vFeatVecRC.lower_bound(Fit->first);
            } else {
                Fit = vFeatVecLK.lower_bound(KFit->first);
            }
        }


        if (mbCheckOrientation) {
            int ind1 = -1;
            int ind2 = -1;
            int ind3 = -1;

            ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

            for (int i = 0; i < HISTO_LENGTH; i++) {
                if (i == ind1 || i == ind2 || i == ind3)
                    continue;
                for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
                    vpMatches[rotHist[i][j]] = static_cast<MapPoint *>(NULL);
                    nmatches--;
                }
            }
        }
        return nmatches;
    }



    int ORBmatcher::SearchByBoW(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches12)
    {
        const vector<cv::KeyPoint> &vKeysUn1 = pKF1->mvKeysUn;
        const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
        int newMatches = 0;
        vector <MapPoint *> pKF1MapPoints;
        const cv::Mat &Descriptors1 = pKF1->mDescriptors;

        if (pKF1->LoadedFromDisk() == true)// if loaded from disk, use indexed map points
        {
            pKF1MapPoints = pKF1->GetIndexedMapPoints();
        }
        else // for current session keyframe, use indexed key points array
        {
            pKF1MapPoints = pKF1->GetMapPointMatches();
        }

        const vector<MapPoint *> vpMapPoints1 = pKF1MapPoints;

        //Loop closure candidate
        const vector<cv::KeyPoint> &vKeysUn2 = pKF2->mvKeysUn;
        const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;
        const cv::Mat &Descriptors2 = pKF2->mDescriptors;


        vector <MapPoint *> pKF2MapPoints;
        if (pKF2->LoadedFromDisk() == true)// if loaded from disk, use indexed map points
        {
            pKF2MapPoints = pKF2->GetIndexedMapPoints();
        }
        else // for current session keyframe, use indexed key points array
        {
            pKF2MapPoints = pKF2->GetMapPointMatches();
        }

        const vector<MapPoint *> vpMapPoints2 = pKF2MapPoints;

        vpMatches12 = vector<MapPoint *>(vpMapPoints1.size(), static_cast<MapPoint *>(NULL));
        vector<bool> vbMatched2(vpMapPoints2.size(), false);

        vector<int> rotHist[HISTO_LENGTH];
        for (int i = 0; i < HISTO_LENGTH; i++)
            rotHist[i].reserve(500);

        const float factor = 1.0f / HISTO_LENGTH;
        int nmatches = 0;
        DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
        DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
        DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
        DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

        int matched = 0;
        int unmatched = 0;

        while (f1it != f1end && f2it != f2end) {
            if (f1it->first == f2it->first) {
                for (size_t i1 = 0, iend1 = f1it->second.size(); i1 < iend1; i1++) {
                    const size_t idx1 = f1it->second[i1];

                    MapPoint *pMP1 = vpMapPoints1[idx1];
                    if (!pMP1)
                        continue;
                    if (pMP1->isBad())
                        continue;
                    const cv::Mat &d1 = Descriptors1.row(idx1);
                    int bestDist1 = 256;
                    int bestIdx2 = -1;
                    int bestDist2 = 256;

                    for (size_t i2 = 0, iend2 = f2it->second.size(); i2 < iend2; i2++) {
                        const size_t idx2 = f2it->second[i2];
                        if (idx2 >= vpMapPoints2.size())
                        {
                            continue;
                        }
                        else;

                        MapPoint *pMP2 = vpMapPoints2[idx2];
                        // TODO: Check this, why are we getting indicies higher than the size of the vector?
                        //Get the MapPoint_Observation for the point as well
                        if (vbMatched2[idx2] || !pMP2)
                            continue;

                        if (pMP2->isBad())
                            continue;

                        MapPoint_Observation mpObs;
                        cv::Mat pMP2_descriptor;

                        if (pKF2->isLoadedFromDisk)
                            pMP2->GetBestDescriptor(pKF2).copyTo(pMP2_descriptor);
                        else
                            pMP2_descriptor = Descriptors2.row(idx2);
                        //Making changes-KPs
                        const cv::Mat &d2 = pMP2_descriptor;
                        int dist = DescriptorDistance(d1, d2);

                        if (dist < bestDist1) {
                            bestDist2 = bestDist1;
                            bestDist1 = dist;
                            bestIdx2 = idx2;
                        } else if (dist < bestDist2) {
                            bestDist2 = dist;
                        }
                    }

                    if (bestDist1 < TH_LOW) {
                        if (static_cast<float>(bestDist1) < mfNNratio * static_cast<float>(bestDist2))
                        {
                            vpMatches12[idx1] = vpMapPoints2[bestIdx2];
                            vbMatched2[bestIdx2] = true;

                            if ( vpMapPoints2 [bestIdx2]->GetFromPreviousSession() == true )
                            {
                                vpMapPoints2 [bestIdx2]->IncrementNumberOfOccurencesInNewSession();
                                newMatches++;
                            }

                            if (mbCheckOrientation)
                            {

                                float angle;
                                if (pKF2->isLoadedFromDisk == true) {
                                    MapPoint *mapPoint = vpMapPoints2[bestIdx2];
                                    angle = mapPoint->mpObservation.find(pKF2)->second.Keypoint.angle;
                                } else {
                                    angle = vKeysUn2[bestIdx2].angle;
                                }

                                float rot = vKeysUn1[idx1].angle - angle;
                                if (rot < 0.0)
                                    rot += 360.0f;
                                int bin = round(rot * factor);
                                if (bin == HISTO_LENGTH)
                                    bin = 0;
                                assert(bin >= 0 && bin < HISTO_LENGTH);
                                rotHist[bin].push_back(idx1);
                            }
                            nmatches++;
                        }
                    }
                }

                f1it++;
                f2it++;
            } else if (f1it->first < f2it->first) {
                f1it = vFeatVec1.lower_bound(f2it->first);
            } else {
                f2it = vFeatVec2.lower_bound(f1it->first);
            }
        }

        if (mbCheckOrientation) {
            int ind1 = -1;
            int ind2 = -1;
            int ind3 = -1;

            ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

            for (int i = 0; i < HISTO_LENGTH; i++) {
                if (i == ind1 || i == ind2 || i == ind3)
                    continue;
                for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
                    vpMatches12[rotHist[i][j]] = static_cast<MapPoint *>(NULL);
                    nmatches--;
                }
            }
        }
        return nmatches;
    }

    int ORBmatcher::SearchForTriangulation(KeyFrame *pKF1, KeyFrame *pKF2, cv::Mat F12,
                                           vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo) {
        const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
        const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;

        unsigned int prevSessionMatches = 0;

        //Compute epipole in second image
        cv::Mat Cw = pKF1->GetCameraCenter();
        cv::Mat R2w = pKF2->GetRotation();
        cv::Mat t2w = pKF2->GetTranslation();
        cv::Mat C2 = R2w * Cw + t2w;
        const float invz = 1.0f / C2.at<float>(2);
        const float ex = pKF2->fx * C2.at<float>(0) * invz + pKF2->cx;
        const float ey = pKF2->fy * C2.at<float>(1) * invz + pKF2->cy;

        // Find matches between not tracked keypoints
        // Matching speed-up by ORB Vocabulary
        // Compare only ORB that share the same node

        int nmatches = 0;
        vector<bool> vbMatched2(pKF2->N, false);
        vector<int> vMatches12(pKF1->N, -1);

        vector<int> rotHist[HISTO_LENGTH];
        for (int i = 0; i < HISTO_LENGTH; i++)
            rotHist[i].reserve(500);

        const float factor = 1.0f / HISTO_LENGTH;

        DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
        DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
        DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
        DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

        while (f1it != f1end && f2it != f2end) {
            if (f1it->first == f2it->first) {
                for (size_t i1 = 0, iend1 = f1it->second.size(); i1 < iend1; i1++) {
                    const size_t idx1 = f1it->second[i1];

                    MapPoint *pMP1 = pKF1->GetMapPoint(idx1);

                    // If there is already a MapPoint skip
                    if (pMP1)
                        continue;

                    const bool bStereo1 = pKF1->mvuRight[idx1] >= 0;

                    if (bOnlyStereo)
                        if (!bStereo1)
                            continue;

                    const cv::KeyPoint &kp1 = pKF1->mvKeysUn[idx1];

                    const cv::Mat &d1 = pKF1->mDescriptors.row(idx1);

                    int bestDist = TH_LOW;
                    int bestIdx2 = -1;

                    for (size_t i2 = 0, iend2 = f2it->second.size(); i2 < iend2; i2++) {
                        size_t idx2 = f2it->second[i2];

                        MapPoint *pMP2 = pKF2->GetMapPoint(idx2);

                        // If we have already matched or there is a MapPoint skip
                        if (vbMatched2[idx2] || pMP2)
                            continue;

                        const bool bStereo2 = pKF2->mvuRight[idx2] >= 0;

                        if (bOnlyStereo)
                            if (!bStereo2)
                                continue;

                        const cv::Mat &d2 = pKF2->mDescriptors.row(idx2);

                        const int dist = DescriptorDistance(d1, d2);

                        if (dist > TH_LOW || dist > bestDist)
                            continue;

                        const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];

                        if (!bStereo1 && !bStereo2) {
                            const float distex = ex - kp2.pt.x;
                            const float distey = ey - kp2.pt.y;
                            if (distex * distex + distey * distey < 100 * pKF2->mvScaleFactors[kp2.octave])
                                continue;
                        }

                        if (CheckDistEpipolarLine(kp1, kp2, F12, pKF2)) {
                            bestIdx2 = idx2;
                            bestDist = dist;
                        }
                    }

                    if (bestIdx2 >= 0) {
                        const cv::KeyPoint &kp2 = pKF2->mvKeysUn[bestIdx2];
                        vMatches12[idx1] = bestIdx2;
                        vbMatched2[bestIdx2] = true;
                        nmatches++;

                        if (mbCheckOrientation) {
                            float rot = kp1.angle - kp2.angle;
                            if (rot < 0.0)
                                rot += 360.0f;
                            int bin = round(rot * factor);
                            if (bin == HISTO_LENGTH)
                                bin = 0;
                            assert(bin >= 0 && bin < HISTO_LENGTH);
                            rotHist[bin].push_back(idx1);
                        }
                    }
                }

                f1it++;
                f2it++;
            } else if (f1it->first < f2it->first) {
                f1it = vFeatVec1.lower_bound(f2it->first);
            } else {
                f2it = vFeatVec2.lower_bound(f1it->first);
            }
        }

        if (mbCheckOrientation) {
            int ind1 = -1;
            int ind2 = -1;
            int ind3 = -1;

            ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

            for (int i = 0; i < HISTO_LENGTH; i++) {
                if (i == ind1 || i == ind2 || i == ind3)
                    continue;
                for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
                    vbMatched2[vMatches12[rotHist[i][j]]] = false;
                    vMatches12[rotHist[i][j]] = -1;
                    nmatches--;
                }
            }

        }

        vMatchedPairs.clear();
        vMatchedPairs.reserve(nmatches);

        for (size_t i = 0, iend = vMatches12.size(); i < iend; i++) {
            if (vMatches12[i] < 0)
                continue;
            vMatchedPairs.push_back(make_pair(i, vMatches12[i]));
        }

        return nmatches;
    }

    int ORBmatcher::Fuse(KeyFrame *pKF, const vector<MapPoint *> &vpMapPoints, const float th) {

        //Project vpMapPoints in pKF
        //1. See if the map point vpMapPoint can exist in pKF
        //2. If yes, then find the most similar key point in pKF according to descriptor distance and spatial relationship
        //3. After that, try to FUSE them

        if (pKF->isLoadedFromDisk) {
            return 0;
        }
        //Find matches of map points present in vpMapPooints in the keyframe pKF
        cv::Mat Rcw = pKF->GetRotation();
        cv::Mat tcw = pKF->GetTranslation();

        const float &fx = pKF->fx;
        const float &fy = pKF->fy;
        const float &cx = pKF->cx;
        const float &cy = pKF->cy;
        const float &bf = pKF->mbf;

        cv::Mat Ow = pKF->GetCameraCenter();

        int nFused = 0;

        const int nMPs = vpMapPoints.size();

        //Project each map point into keyframe *pKF
        //Then search for key point matches in *pKF
        for (int i = 0; i < nMPs; i++) {
            MapPoint *pMP = vpMapPoints[i];

            if (!pMP)
                continue;

            if (pMP->isBad() || pMP->IsInKeyFrame(pKF))
                continue;

            cv::Mat p3Dw = pMP->GetWorldPos();
            cv::Mat p3Dc = Rcw * p3Dw + tcw;

            // Depth must be positive
            if (p3Dc.at<float>(2) < 0.0f)
                continue;

            const float invz = 1 / p3Dc.at<float>(2);
            const float x = p3Dc.at<float>(0) * invz;
            const float y = p3Dc.at<float>(1) * invz;

            const float u = fx * x + cx;
            const float v = fy * y + cy;

            // Point must be inside the image
            if (!pKF->IsInImage(u, v))
                continue;

            const float ur = u - bf * invz;

            const float maxDistance = pMP->GetMaxDistanceInvariance();
            const float minDistance = pMP->GetMinDistanceInvariance();
            cv::Mat PO = p3Dw - Ow;
            const float dist3D = cv::norm(PO);

            // Depth must be inside the scale pyramid of the image
            if (dist3D < minDistance || dist3D > maxDistance)
                continue;

            // Viewing angle must be less than 60 deg
            cv::Mat Pn = pMP->GetNormal();

            if (PO.dot(Pn) < 0.5 * dist3D)
                continue;

            int nPredictedLevel = pMP->PredictScale(dist3D, pKF);

            // Search in a radius
            const float radius = th * pKF->mvScaleFactors[nPredictedLevel];

            const vector<size_t> vIndices = pKF->GetFeaturesInArea(u, v, radius);

            if (vIndices.empty())
                continue;

            // Match to the most similar keypoint in the radius

            const cv::Mat dMP = pMP->GetDescriptor();

            int bestDist = 256;
            int bestIdx = -1;
            for (vector<size_t>::const_iterator vit = vIndices.begin(), vend = vIndices.end(); vit != vend; vit++) {
                const size_t idx = *vit;

                const cv::KeyPoint &kp = pKF->mvKeysUn[idx];

                const int &kpLevel = kp.octave;

                if (kpLevel < nPredictedLevel - 1 || kpLevel > nPredictedLevel)
                    continue;

                if (pKF->mvuRight[idx] >= 0) {
                    // Check reprojection error in stereo
                    const float &kpx = kp.pt.x;
                    const float &kpy = kp.pt.y;
                    const float &kpr = pKF->mvuRight[idx];
                    const float ex = u - kpx;
                    const float ey = v - kpy;
                    const float er = ur - kpr;
                    const float e2 = ex * ex + ey * ey + er * er;

                    if (e2 * pKF->mvInvLevelSigma2[kpLevel] > 7.8)
                        continue;
                } else {
                    const float &kpx = kp.pt.x;
                    const float &kpy = kp.pt.y;
                    const float ex = u - kpx;
                    const float ey = v - kpy;
                    const float e2 = ex * ex + ey * ey;

                    if (e2 * pKF->mvInvLevelSigma2[kpLevel] > 5.99)
                        continue;
                }

                const cv::Mat &dKF = pKF->mDescriptors.row(idx);

                const int dist = DescriptorDistance(dMP, dKF);

                if (dist < bestDist) {
                    bestDist = dist;
                    bestIdx = idx;
                }
            }

            //bestIdx contains the index of the key point that is closest in terms of visual and spatial relationship
            //to the current map point (this)
            if (bestDist <= TH_LOW) {
                MapPoint *pMPinKF = pKF->GetMapPoint(bestIdx);
                if (pMP->GetFromPreviousSession() == true)
                {
                    pMP->IncrementNumberOfOccurencesInNewSession();
                }

                if (pMPinKF != NULL)
                    if (pMPinKF->GetFromPreviousSession() == true)
                    {
                        pMPinKF->IncrementNumberOfOccurencesInNewSession();
                    }

                if (pMPinKF)//if pMPinKF is already a map point
                {
                    if (!pMPinKF->isBad())
                    {
                        //Then replace one with another to remove duplicates
                        if (pMPinKF->Observations() > pMP->Observations())
                        {
                            pMP->Replace(pMPinKF);
                        }
                        else
                        {
                            pMPinKF->Replace(pMP);
                        }
                    }
                }
                else//If it is not a map point, then add it as a map point
                    //i.e, the current map point was also observed in pKF at bestIDX
                {
                    pMP->AddObservation(pKF, bestIdx);
                    pKF->AddMapPoint(pMP, bestIdx);
                }
                nFused++;
            }
        }
        return nFused;
    }

    int ORBmatcher::Fuse(KeyFrame *pKF, cv::Mat Scw, const vector<MapPoint *> &vpPoints, float th,
                         vector<MapPoint *> &vpReplacePoint) {
        // Get Calibration Parameters for later projection
        const float &fx = pKF->fx;
        const float &fy = pKF->fy;
        const float &cx = pKF->cx;
        const float &cy = pKF->cy;

        // Decompose Scw
        cv::Mat sRcw = Scw.rowRange(0, 3).colRange(0, 3);
        const float scw = sqrt(sRcw.row(0).dot(sRcw.row(0)));
        cv::Mat Rcw = sRcw / scw;
        cv::Mat tcw = Scw.rowRange(0, 3).col(3) / scw;
        cv::Mat Ow = -Rcw.t() * tcw;

        // Set of MapPoints already found in the KeyFrame
        const set<MapPoint *> spAlreadyFound = pKF->GetMapPoints();

        int nFused = 0;

        const int nPoints = vpPoints.size();

        // For each candidate MapPoint project and match
        for (int iMP = 0; iMP < nPoints; iMP++) {
            MapPoint *pMP = vpPoints[iMP];

            // Discard Bad MapPoints and already found
            if (pMP->isBad() || spAlreadyFound.count(pMP))
                continue;

            // Get 3D Coords.
            cv::Mat p3Dw = pMP->GetWorldPos();

            // Transform into Camera Coords.
            cv::Mat p3Dc = Rcw * p3Dw + tcw;

            // Depth must be positive
            if (p3Dc.at<float>(2) < 0.0f)
                continue;

            // Project into Image
            const float invz = 1.0 / p3Dc.at<float>(2);
            const float x = p3Dc.at<float>(0) * invz;
            const float y = p3Dc.at<float>(1) * invz;

            const float u = fx * x + cx;
            const float v = fy * y + cy;

            // Point must be inside the image
            if (!pKF->IsInImage(u, v))
                continue;

            // Depth must be inside the scale pyramid of the image
            const float maxDistance = pMP->GetMaxDistanceInvariance();
            const float minDistance = pMP->GetMinDistanceInvariance();
            cv::Mat PO = p3Dw - Ow;
            const float dist3D = cv::norm(PO);

            if (dist3D < minDistance || dist3D > maxDistance)
                continue;

            // Viewing angle must be less than 60 deg
            cv::Mat Pn = pMP->GetNormal();

            if (PO.dot(Pn) < 0.5 * dist3D)
                continue;

            // Compute predicted scale level
            const int nPredictedLevel = pMP->PredictScale(dist3D, pKF);

            // Search in a radius
            const float radius = th * pKF->mvScaleFactors[nPredictedLevel];

            if (pKF->LoadedFromDisk()) {
                cerr << "Another problem here" << endl;
                continue;
            }

            const vector<size_t> vIndices = pKF->GetFeaturesInArea(u, v, radius);

            if (vIndices.empty())
                continue;

            // Match to the most similar keypoint in the radius

            const cv::Mat dMP = pMP->GetDescriptor();

            int bestDist = INT_MAX;
            int bestIdx = -1;
            for (vector<size_t>::const_iterator vit = vIndices.begin(); vit != vIndices.end(); vit++) {
                const size_t idx = *vit;
                const int &kpLevel = pKF->mvKeysUn[idx].octave;

                if (kpLevel < nPredictedLevel - 1 || kpLevel > nPredictedLevel)
                    continue;

                const cv::Mat &dKF = pKF->mDescriptors.row(idx);

                int dist = DescriptorDistance(dMP, dKF);

                if (dist < bestDist) {
                    bestDist = dist;
                    bestIdx = idx;
                }
            }

            // If there is already a MapPoint replace otherwise add new measurement
            if (bestDist <= TH_LOW) {

                if (pMP->GetFromPreviousSession() == true)
                    pMP->IncrementNumberOfOccurencesInNewSession();

                MapPoint *pMPinKF = pKF->GetMapPoint(bestIdx);
                if (pMPinKF) {

                    if (pMPinKF->GetFromPreviousSession())
                        pMPinKF->IncrementNumberOfOccurencesInNewSession();

                    if (!pMPinKF->isBad())
                        vpReplacePoint[iMP] = pMPinKF;
                } else {
                    pMP->AddObservation(pKF, bestIdx);
                    pKF->AddMapPoint(pMP, bestIdx);
                }
                nFused++;
            }
        }

        return nFused;
    }

    int ORBmatcher::SearchBySim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches12,
                                 const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th) {
        //Get camera intrinsics for current keyframe
        //Find the transformation between camera 1 and camera 2
        //Figure out how many map points and WHICH map points are already matched in the two keyframes
        //Store these in vbAlreadyMatched1 and vbAlreadyMatched2
        //Iterate through all map points in current keyframe and try to project them in the loop closure candidate
        //  - Skip those which are already matched
        //  - First make sure that the map point can exist in the loop candidate by projecting it and checking geometrical constraints
        //  - All it checks out, then try to find the key-points in its geometric proximity and try to match them
        //  - Using this, try to find the closest matches for map-points from current keyframe in loop closure candidate
        //  - In our system, if the loop closure candidate is loaded from disk, then we skip this whole process because we don't have key-points

        //Then, iterate through all map points in the loop closure candidate and try to match them with key-points from the current key-frame
        //  - Do the exact same process to find more matches
        //  - We can do matching in this process because we have all key-points for the current keyframe

        //Once all this is done, return the number of matches


        const float &fx = pKF1->fx;
        const float &fy = pKF1->fy;
        const float &cx = pKF1->cx;
        const float &cy = pKF1->cy;

        // Camera 1 from world
        cv::Mat R1w = pKF1->GetRotation();
        cv::Mat t1w = pKF1->GetTranslation();

        //Camera 2 from world
        cv::Mat R2w = pKF2->GetRotation();
        cv::Mat t2w = pKF2->GetTranslation();

        //Transformation between cameras
        cv::Mat sR12 = s12 * R12;
        cv::Mat sR21 = (1.0 / s12) * R12.t();
        cv::Mat t21 = -sR21 * t12;

        const vector<MapPoint *> vpMapPoints1 = pKF1->GetMapPointMatches();
        const int N1 = vpMapPoints1.size();

        const vector<MapPoint *> vpMapPoints2 = pKF2->GetMapPointMatches();
        const int N2 = vpMapPoints2.size();

        vector<bool> vbAlreadyMatched1(N1, false);
        vector<bool> vbAlreadyMatched2(N2, false);

        for (int i = 0; i < N1; i++) {
            MapPoint *pMP = vpMatches12[i];
            if (pMP) {
                vbAlreadyMatched1[i] = true;
                int idx2 = pMP->GetIndexInKeyFrame(pKF2);
                if (idx2 >= 0 && idx2 < N2)
                    vbAlreadyMatched2[idx2] = true;
            }
        }

        vector<int> vnMatch1(N1, -1);
        vector<int> vnMatch2(N2, -1);

        // Transform from KF1 to KF2 and search
        for (int i1 = 0; i1 < N1; i1++) {
            MapPoint *pMP = vpMapPoints1[i1];

            if (!pMP || vbAlreadyMatched1[i1])
                continue;

            if (pMP->isBad())
                continue;

            cv::Mat p3Dw = pMP->GetWorldPos();
            cv::Mat p3Dc1 = R1w * p3Dw + t1w;
            cv::Mat p3Dc2 = sR21 * p3Dc1 + t21;

            // Depth must be positive
            if (p3Dc2.at<float>(2) < 0.0)
                continue;

            const float invz = 1.0 / p3Dc2.at<float>(2);
            const float x = p3Dc2.at<float>(0) * invz;
            const float y = p3Dc2.at<float>(1) * invz;

            const float u = fx * x + cx;
            const float v = fy * y + cy;

            // Point must be inside the image
            if (!pKF2->IsInImage(u, v))
                continue;

            const float maxDistance = pMP->GetMaxDistanceInvariance();
            const float minDistance = pMP->GetMinDistanceInvariance();
            const float dist3D = cv::norm(p3Dc2);

            // Depth must be inside the scale invariance region
            if (dist3D < minDistance || dist3D > maxDistance)
                continue;

            // Compute predicted octave
            const int nPredictedLevel = pMP->PredictScale(dist3D, pKF2);

            // Search in a radius
            const float radius = th * pKF2->mvScaleFactors[nPredictedLevel];


            //Making changes-KPs

            //If the loop closure candidate is loaded from disk, there's no need to bother trying to search in a given area for a match
            if (pKF2->isLoadedFromDisk == true)
                continue;

            const vector<size_t> vIndices = pKF2->GetFeaturesInArea(u, v, radius);

            if (vIndices.empty())
                continue;

            // Match to the most similar keypoint in the radius
            const cv::Mat dMP = pMP->GetDescriptor();

            int bestDist = INT_MAX;
            int bestIdx = -1;
            for (vector<size_t>::const_iterator vit = vIndices.begin(), vend = vIndices.end(); vit != vend; vit++) {
                const size_t idx = *vit;

                const cv::KeyPoint &kp = pKF2->mvKeysUn[idx];

                if (kp.octave < nPredictedLevel - 1 || kp.octave > nPredictedLevel)
                    continue;

                const cv::Mat &dKF = pKF2->mDescriptors.row(idx);

                const int dist = DescriptorDistance(dMP, dKF);

                if (dist < bestDist) {
                    bestDist = dist;
                    bestIdx = idx;
                }
            }

            if (bestDist <= TH_HIGH) {
                vnMatch1[i1] = bestIdx;
            }
        }

        // Transform from KF2 to KF2 and search
        for (int i2 = 0; i2 < N2; i2++) {
            MapPoint *pMP = vpMapPoints2[i2];

            if (!pMP || vbAlreadyMatched2[i2])
                continue;

            if (pMP->isBad())
                continue;

            cv::Mat p3Dw = pMP->GetWorldPos();
            cv::Mat p3Dc2 = R2w * p3Dw + t2w;
            cv::Mat p3Dc1 = sR12 * p3Dc2 + t12;

            // Depth must be positive
            if (p3Dc1.at<float>(2) < 0.0)
                continue;

            const float invz = 1.0 / p3Dc1.at<float>(2);
            const float x = p3Dc1.at<float>(0) * invz;
            const float y = p3Dc1.at<float>(1) * invz;

            const float u = fx * x + cx;
            const float v = fy * y + cy;

            // Point must be inside the image
            if (!pKF1->IsInImage(u, v))
                continue;

            const float maxDistance = pMP->GetMaxDistanceInvariance();
            const float minDistance = pMP->GetMinDistanceInvariance();
            const float dist3D = cv::norm(p3Dc1);

            // Depth must be inside the scale pyramid of the image
            if (dist3D < minDistance || dist3D > maxDistance)
                continue;

            // Compute predicted octave
            const int nPredictedLevel = pMP->PredictScale(dist3D, pKF1);

            // Search in a radius of 2.5*sigma(ScaleLevel)
            const float radius = th * pKF1->mvScaleFactors[nPredictedLevel];

            if (pKF1->LoadedFromDisk())
                cerr << "Problem here" << endl;

            const vector<size_t> vIndices = pKF1->GetFeaturesInArea(u, v, radius);

            if (vIndices.empty())
                continue;

            // Match to the most similar keypoint in the radius
            const cv::Mat dMP = pMP->GetDescriptor();

            int bestDist = INT_MAX;
            int bestIdx = -1;
            for (vector<size_t>::const_iterator vit = vIndices.begin(), vend = vIndices.end(); vit != vend; vit++) {
                const size_t idx = *vit;

                const cv::KeyPoint &kp = pKF1->mvKeysUn[idx];

                if (kp.octave < nPredictedLevel - 1 || kp.octave > nPredictedLevel)
                    continue;

                const cv::Mat &dKF = pKF1->mDescriptors.row(idx);

                const int dist = DescriptorDistance(dMP, dKF);

                if (dist < bestDist) {
                    bestDist = dist;
                    bestIdx = idx;
                }
            }

            if (bestDist <= TH_HIGH) {
                vnMatch2[i2] = bestIdx;
            }
        }

        // Check agreement
        int nFound = 0;

        for (int i1 = 0; i1 < N1; i1++) {
            int idx2 = vnMatch1[i1];

            if (idx2 >= 0) {
                int idx1 = vnMatch2[idx2];
                if (idx1 == i1) {
                    vpMatches12[i1] = vpMapPoints2[idx2];
                    nFound++;
                }
            }
        }

        return nFound;
    }

    int ORBmatcher::SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono) {
        int nmatches = 0;
        int prevSessionMatches = 0;

        const bool isDebugMode = false;

        // Rotation Histogram (to check rotation consistency)
        vector<int> rotHist[HISTO_LENGTH];
        for (int i = 0; i < HISTO_LENGTH; i++)
            rotHist[i].reserve(500);
        const float factor = 1.0f / HISTO_LENGTH;

        const cv::Mat Rcw = CurrentFrame.mTcw.rowRange(0, 3).colRange(0, 3);
        const cv::Mat tcw = CurrentFrame.mTcw.rowRange(0, 3).col(3);

        const cv::Mat twc = -Rcw.t() * tcw;

        const cv::Mat Rlw = LastFrame.mTcw.rowRange(0, 3).colRange(0, 3);
        const cv::Mat tlw = LastFrame.mTcw.rowRange(0, 3).col(3);

        const cv::Mat tlc = Rlw * twc + tlw;

        const bool bForward = tlc.at<float>(2) > CurrentFrame.mb && !bMono;
        const bool bBackward = -tlc.at<float>(2) > CurrentFrame.mb && !bMono;

        for (int i = 0; i < LastFrame.N; i++) {
            MapPoint *pMP = LastFrame.mvpMapPoints[i];

            if (pMP) {
                if (!LastFrame.mvbOutlier[i]) {
                    // Project
                    cv::Mat x3Dw = pMP->GetWorldPos();
                    cv::Mat x3Dc = Rcw * x3Dw + tcw;

                    const float xc = x3Dc.at<float>(0);
                    const float yc = x3Dc.at<float>(1);
                    const float invzc = 1.0 / x3Dc.at<float>(2);

                    if (invzc < 0)
                        continue;

                    float u = CurrentFrame.fx * xc * invzc + CurrentFrame.cx;
                    float v = CurrentFrame.fy * yc * invzc + CurrentFrame.cy;

                    if (u < CurrentFrame.mnMinX || u > CurrentFrame.mnMaxX)
                        continue;
                    if (v < CurrentFrame.mnMinY || v > CurrentFrame.mnMaxY)
                        continue;

                    int nLastOctave = LastFrame.mvKeys[i].octave;

                    // Search in a window. Size depends on scale
                    float radius = th * CurrentFrame.mvScaleFactors[nLastOctave];

                    vector<size_t> vIndices2;

                    if (bForward)
                        vIndices2 = CurrentFrame.GetFeaturesInArea(u, v, radius, nLastOctave);
                    else if (bBackward)
                        vIndices2 = CurrentFrame.GetFeaturesInArea(u, v, radius, 0, nLastOctave);
                    else
                        vIndices2 = CurrentFrame.GetFeaturesInArea(u, v, radius, nLastOctave - 1, nLastOctave + 1);

                    if (vIndices2.empty())
                        continue;

                    const cv::Mat dMP = pMP->GetDescriptor();

                    int bestDist = 256;
                    int bestIdx2 = -1;

                    for (vector<size_t>::const_iterator vit = vIndices2.begin(), vend = vIndices2.end();
                         vit != vend; vit++) {
                        const size_t i2 = *vit;
                        if (CurrentFrame.mvpMapPoints[i2])
                            if (CurrentFrame.mvpMapPoints[i2]->Observations() > 0)
                                continue;

                        if (CurrentFrame.mvuRight[i2] > 0) {
                            const float ur = u - CurrentFrame.mbf * invzc;
                            const float er = fabs(ur - CurrentFrame.mvuRight[i2]);
                            if (er > radius)
                                continue;
                        }

                        const cv::Mat &d = CurrentFrame.mDescriptors.row(i2);

                        const int dist = DescriptorDistance(dMP, d);

                        if (dist < bestDist) {
                            bestDist = dist;
                            bestIdx2 = i2;
                        }
                    }

                    if (bestDist <= TH_HIGH) {
                        CurrentFrame.mvpMapPoints[bestIdx2] = pMP;
                        nmatches++;

                        if (majorityVoting)
                            pMP->AppendLabel (CurrentFrame.mvLabels[bestIdx2]);

                        if (pMP->GetFromPreviousSession() == true) {
                            if (isDebugMode)
                                cout << "SearchByProjectin(): pMP = " << pMP->mnId
                                     << " from the previous session has been seen again" << endl;
                            pMP->IncrementNumberOfOccurencesInNewSession();
                            prevSessionMatches++;
                        }

                        if (mbCheckOrientation) {
                            float rot = LastFrame.mvKeysUn[i].angle - CurrentFrame.mvKeysUn[bestIdx2].angle;
                            if (rot < 0.0)
                                rot += 360.0f;
                            int bin = round(rot * factor);
                            if (bin == HISTO_LENGTH)
                                bin = 0;
                            assert(bin >= 0 && bin < HISTO_LENGTH);
                            rotHist[bin].push_back(bestIdx2);


                        }
                    }
                }
            }
        }

        //Apply rotation consistency
        if (mbCheckOrientation) {
            int ind1 = -1;
            int ind2 = -1;
            int ind3 = -1;

            ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

            for (int i = 0; i < HISTO_LENGTH; i++) {
                if (i != ind1 && i != ind2 && i != ind3) {
                    for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
                        CurrentFrame.mvpMapPoints[rotHist[i][j]] = static_cast<MapPoint *>(NULL);
                        if (isDebugMode)
                            cout << "Removing mp = " << rotHist[i][j] << endl;
                        nmatches--;
                    }
                }
            }
        }
        return nmatches;
    }

    int ORBmatcher::SearchByProjection(Frame &CurrentFrame, KeyFrame *pKF, const set<MapPoint *> &sAlreadyFound,
                                       const float th, const int ORBdist) {
        int nmatches = 0;
        unsigned int prevSessionMatches = 0;
        const bool isDebugMode = false;

        const cv::Mat Rcw = CurrentFrame.mTcw.rowRange(0, 3).colRange(0, 3);
        const cv::Mat tcw = CurrentFrame.mTcw.rowRange(0, 3).col(3);
        const cv::Mat Ow = -Rcw.t() * tcw;

        // Rotation Histogram (to check rotation consistency)
        vector<int> rotHist[HISTO_LENGTH];
        for (int i = 0; i < HISTO_LENGTH; i++)
            rotHist[i].reserve(500);
        const float factor = 1.0f / HISTO_LENGTH;

        const vector<MapPoint *> vpMPs = pKF->GetMapPointMatches();

        for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
            MapPoint *pMP = vpMPs[i];

            if (pMP) {
                if (!pMP->isBad() && !sAlreadyFound.count(pMP)) {
                    //Project
                    cv::Mat x3Dw = pMP->GetWorldPos();
                    cv::Mat x3Dc = Rcw * x3Dw + tcw;

                    const float xc = x3Dc.at<float>(0);
                    const float yc = x3Dc.at<float>(1);
                    const float invzc = 1.0 / x3Dc.at<float>(2);

                    const float u = CurrentFrame.fx * xc * invzc + CurrentFrame.cx;
                    const float v = CurrentFrame.fy * yc * invzc + CurrentFrame.cy;

                    if (u < CurrentFrame.mnMinX || u > CurrentFrame.mnMaxX)
                        continue;
                    if (v < CurrentFrame.mnMinY || v > CurrentFrame.mnMaxY)
                        continue;

                    // Compute predicted scale level
                    cv::Mat PO = x3Dw - Ow;
                    float dist3D = cv::norm(PO);

                    const float maxDistance = pMP->GetMaxDistanceInvariance();
                    const float minDistance = pMP->GetMinDistanceInvariance();

                    // Depth must be inside the scale pyramid of the image
                    if (dist3D < minDistance || dist3D > maxDistance)
                        continue;

                    int nPredictedLevel = pMP->PredictScale(dist3D, &CurrentFrame);

                    // Search in a window
                    const float radius = th * CurrentFrame.mvScaleFactors[nPredictedLevel];

                    const vector<size_t> vIndices2 = CurrentFrame.GetFeaturesInArea(u, v, radius, nPredictedLevel - 1,
                                                                                    nPredictedLevel + 1);

                    if (vIndices2.empty())
                        continue;

                    const cv::Mat dMP = pMP->GetDescriptor();

                    int bestDist = 256;
                    int bestIdx2 = -1;

                    for (vector<size_t>::const_iterator vit = vIndices2.begin(); vit != vIndices2.end(); vit++) {
                        const size_t i2 = *vit;
                        if (CurrentFrame.mvpMapPoints[i2])
                            continue;

                        const cv::Mat &d = CurrentFrame.mDescriptors.row(i2);

                        const int dist = DescriptorDistance(dMP, d);

                        if (dist < bestDist) {
                            bestDist = dist;
                            bestIdx2 = i2;
                        }
                    }

                    if (bestDist <= ORBdist) {
                        CurrentFrame.mvpMapPoints[bestIdx2] = pMP;
                        nmatches++;

                        if (pMP->GetFromPreviousSession() == true)
                        {
                            pMP->IncrementNumberOfOccurencesInNewSession();
                            prevSessionMatches++;
                        }

                        if (mbCheckOrientation) {
                            //Making changes-KP
                            //SearchByProjection - Rot
                            float ROT = pMP->unKP.find(pKF)->second.angle - CurrentFrame.mvKeysUn[bestIdx2].angle;
                            float rot = pMP->mpObservation.find(pKF)->second.Keypoint.angle -
                                        CurrentFrame.mvKeysUn[bestIdx2].angle;

                            if (rot < 0.0)
                                rot += 360.0f;
                            int bin = round(rot * factor);
                            if (bin == HISTO_LENGTH)
                                bin = 0;
                            assert(bin >= 0 && bin < HISTO_LENGTH);
                            rotHist[bin].push_back(bestIdx2);
                        }
                    }
                }
            }
        }

        if (mbCheckOrientation) {
            int ind1 = -1;
            int ind2 = -1;
            int ind3 = -1;

            ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

            for (int i = 0; i < HISTO_LENGTH; i++) {
                if (i != ind1 && i != ind2 && i != ind3) {
                    for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
                        CurrentFrame.mvpMapPoints[rotHist[i][j]] = NULL;
                        nmatches--;
                    }
                }
            }
        }

        return nmatches;
    }


    void ORBmatcher::ComputeThreeMaxima(vector<int> *histo, const int L, int &ind1, int &ind2, int &ind3) {
        int max1 = 0;
        int max2 = 0;
        int max3 = 0;

        for (int i = 0; i < L; i++) {
            const int s = histo[i].size();
            if (s > max1) {
                max3 = max2;
                max2 = max1;
                max1 = s;
                ind3 = ind2;
                ind2 = ind1;
                ind1 = i;
            } else if (s > max2) {
                max3 = max2;
                max2 = s;
                ind3 = ind2;
                ind2 = i;
            } else if (s > max3) {
                max3 = s;
                ind3 = i;
            }
        }

        if (max2 < 0.1f * (float) max1) {
            ind2 = -1;
            ind3 = -1;
        } else if (max3 < 0.1f * (float) max1) {
            ind3 = -1;
        }
    }
// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
    int ORBmatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
    {
        const int *pa = a.ptr<int32_t>();
        const int *pb = b.ptr<int32_t>();

        int dist = 0;

        for (int i = 0; i < 8; i++, pa++, pb++) {
            unsigned int v = *pa ^*pb;
            v = v - ((v >> 1) & 0x55555555);
            v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
            dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
        }

        return dist;
    }

} //namespace ORB_SLAM
