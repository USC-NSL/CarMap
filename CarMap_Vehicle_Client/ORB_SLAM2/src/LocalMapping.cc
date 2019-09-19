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

#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "MapPointObservation.h"

#include "SemanticSegmentor.h"
#include<mutex>

namespace ORB_SLAM2
{

LocalMapping::LocalMapping(Map *pMap, const float bMonocular):
    mbMonocular(bMonocular), mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true), stitchingMode (false)
{
}

    LocalMapping::LocalMapping(Map *pMap, const float bMonocular, const bool isStitching, const bool isDynamicObjectRemoval, bool majorityVoting):
            mbMonocular(bMonocular), mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
            mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true), stitchingMode (isStitching),
            mIsDynamicObjectRemoval (isDynamicObjectRemoval), MajorityVoting (majorityVoting)
    {
//        cout << "Done with mapping" << endl;
    }

void LocalMapping::SetLoopCloser(LoopClosing* pLoopCloser)
{
    mpLoopCloser = pLoopCloser;
}

void LocalMapping::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void LocalMapping::Run()
{

    bool isDebugMode = false;
    mbFinished = false;
//    bool skip_mapping, skip_BA, skip_culling;
//    if (QS_State == SAVE_MAP)
//    {
//        skip_mapping = false;
//        skip_BA = false;
//        skip_culling = false;
//    } else
//    {
//        skip_mapping = false;
//        skip_BA = false;
//        skip_culling = false;
//    }


    if (stitchingMode)
    {
        while (1)
        {
            if(Stop())
            {
                // Safe area to stop
                while(isStopped() && !CheckFinish())
                {
                    std::this_thread::sleep_for(std::chrono::microseconds(3000));
                }
                if(CheckFinish())
                    break;

                ResetIfRequested();
                SetAcceptKeyFrames(true);
                if(CheckFinish())
                    break;
                std::this_thread::sleep_for(std::chrono::microseconds(3000));

            }

            SetFinish();
            }

        }

    while(1)
    {


        int kfMnID;
        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(false);
        // Check if there are keyframes in the queue

        if(CheckNewKeyFrames())
        {
            if (isDebugMode)
            {
                kfMnID = mlNewKeyFrames.front()->GetmnId();
                cout << "Mapping: Processing keyframe = " << kfMnID << endl;
            }
            // BoW conversion and insertion in Map
            ProcessNewKeyFrame();
            //1. Compute the BoW vector for new KF using their descriptors
            //2. Process all new map points into the keyframe:

                //Add an observation point of that map point for current keyframe
                //Find its normal and depth
                //Compute its descriptors
                //Store its std::map <KeyFrame*, cv::KeyPoint> match

            //3. Update links in the co-visibility graph
            //4, Add keyframe to the map (mpMap)

            if (isDebugMode)
            {
                cout << "Mapping: Culling map-points for keyframe = " << kfMnID << endl;
            }
            // Check recent MapPoints
//            cout << "Skipping map point culling for now" << endl;
            MapPointCulling();
            //1. Remove points if they do not satisfy the necessary requirement: mentioned in the paper
            //Conditions:
                //i. Map point must be visible in at least 25% of the frames it was predicted to be visible in
                //ii. If more the one keyframe has passed since map point creation, it must be observed in at least 3


            // Triangulate new MapPoints
            // Creates new points by MATCHING un-matched KEY POINTS from the current keyframe with neighboring keyframes found from the co-visibility graph
//            if (skip_mapping == false) {
            if (isDebugMode)
            {
                cout << "Mapping: Creating new map-points for keyframe = " << kfMnID << endl;
            }
            CreateNewMapPoints();
//                cout << "Created new map points" << endl;
//            }

            //WE SKIP KEYFRAMES THAT ARE LOADED FROM THE DISK!

            //1. Retrieve all neighbor keyframes (10) for the current keyframe from the co-visibility graph (vpNeighKFs)
            //2. Set an ORB matches but with a smaller threshold i.e., 0.6 instead of 0.75
            //3. For every neighboring keyframe to the current one:
                //i. Compute the fundamental matrix between the current keyframe and the neighboring keyframe
                //--- Fundamental matrix describes the geometrical relationship between the two keyframes ---
                //ii. SearchForTriangulation (): - compare each key point for the two keyframes against each other and store correspondences in vMatchedIndicies, also make sure they fulfil the epipolar constraint
                    //---This is where ORB-SLAM2 looks for key-points that have not been tracked ---
                    //a. Get feature vectors for both keyframes
                    //--- Feature vectors are organized according to some ORB vocabulary levels
                    //--- for faster matching in the same nodes. The feature vectors contains indices of the
                    //--- key points and their descriptors ---
                    //--- Speed up the matching by using ORB that share the same node ---
                    //b. Search through the feature vectors of both frames and try to match keypoints
                    //c. If a map point for the key-point already exists, then skip it and move onto the next one
                    //d. Other checks include: match keypoints that have positive depths
                    //--- As such, we will need the descriptors of all keypoints for this to work ---
                    //e. For each key point in our current keyframe
                        //i. For each key point in the neighboring keyframe
                            //1. Get the next key point from the current keyframe
                            //2. If its a map point or the depth is less than 0, then continue
                            //3. Else, retrieve its cv::KeyPoint and descriptor
                            //4. For the neighboring keyframe
                            //5. Do the exact same but also checked if key point is matched
                            //6. Find the distance between the two descriptors
                            //7. If its too much, then just skip
                            //8. Else, check if they lie on the same epipolar line
                            //9. Do other checks are well e.g., parallax, scale consistency etc.
                            //--- So compare every single point from KF1 to KF2 and choose the one it matches best w.r.t to its descriptor distance ---
                    //f. After matching, we have all the KP to KP matches between the two keyframes
                //iii. Now triangulate these to find their depths and make sure they pass all tests: test include parallax, consistency, scale etc.
                //iv. Find their x3D etc using whatever triangulation method is in use
            //6. Triangulate the matched indices and add them as key points in the current keyframe
            //7. Compute all the necessary meta-data for it, normal depth and descriptors etc
            //8. Triangulation is successful! Create the map point, add the two keyframes as observations

            if(!CheckNewKeyFrames())
            {
                // Find more matches in neighbor keyframes and fuse point duplications
//                if (skip_mapping == false) {
                SearchInNeighbors();
//                }
                //1. Get the 10 closest keyframes from the co-visibility graph to the current keyframe in vpTargetKFs
                //2. For each vpTargetKFs, get 5 closest neighboring keyframes of the closet keyframes as well in vpTargetKFs
                //3. Get all map point matches from the current KF in vpMapPointMatches
                //4. For each matching keyframe:
                    //i. Fuse (matching keyframe, vpMapPointMatches)
                    //--- do a fuse operation between current keyframe's map point matches and all matching keyframes key points and map points to consolidate them ---
                        //a. For each map point match
                            //1. Skip the map point if any of the following checks fail:
                            //--- positive depth, inside image bounds, scale pyramid, viewing angle < 60 wrt to the neighboring keyframe ---
                            //--- map point already exists in the neighboring keyframe ----
                            //2. If all these checks are passed, it means the point is visible in neighboring KF
                            //3. Get KP’s within a specific 2D radius of the current map point in neighboring KF
                            //4. For all KPs that correspond to the map point
                            //--- Find the most similar using descriptor distance and link them ---
                //5. Do the opposite as well, Fuse () from target KF to current KF
                    //--- For this, we find map point matches of other keyframe's in the current keyframe's key points

                //6. After this, update all map points

            }

            mbAbortBA = false;



//            if (skip_BA == false) {
                if (!CheckNewKeyFrames() && !stopRequested())
                {

                    // Local BA
                    if (mpMap->KeyFramesInMap() > 2) {
//                        cout << "BA" << endl;
//                        if (isDebugMode == true)
//                            cout << "Mapping: Skipping local bundle adjustment for = " << kfMnID << endl;
                            Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame, &mbAbortBA, mpMap);
                    }
                    // Check redundant local Keyframes
//                    if (skip_culling == false)
                    if (isDebugMode == true)
                        cout << "Mapping: Culling the keyframe now" << endl;
                        KeyFrameCulling();
                    if (isDebugMode)
                        cout << "Mapping: Done with keyframe culling" << endl;
                }
//            }
//            else
//                cout << "Skipping bundle adjustment" << endl;

            if (isDebugMode)
                cout << "Mapping: Inserting keyframe in mp loop closer" << endl;

                mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
            if (isDebugMode)
                cout << "Mapping: Done with keyframe insertion in mp loop closer" << endl;
        }
        else if(Stop())
        {
            // Safe area to stop
            while(isStopped() && !CheckFinish())
            {
                std::this_thread::sleep_for(std::chrono::microseconds(3000));
            }
            if(CheckFinish())
                break;
        }

        ResetIfRequested();

        // Tracking will see that Local Mapping is busy
//        if (isDebugMode)
//            cout << "Mapping: Accepting new keyframes now" << endl;
        SetAcceptKeyFrames(true);

        if(CheckFinish())
            break;

        std::this_thread::sleep_for(std::chrono::microseconds(3000));
//        if (isDebugMode)
//            cout << "Mapping: Wait for 3 seconds, now onto the next keyframe" << endl;


    }

    SetFinish();
}

    void LocalMapping::LocalBA (KeyFrame *pKF)
    {
        cout << "Running local BA on kf = " << pKF->GetmnId() << endl;
        Optimizer::CustomBA(pKF, &mbAbortBA, mpMap);
    }

void LocalMapping::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
    mbAbortBA=true;
}


bool LocalMapping::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}


    bool LocalMapping::CheckAndProcessNewKeyFrame()
    {
        if (CheckNewKeyFrames())
            ProcessNewKeyFrame();

    }

void LocalMapping::ProcessNewKeyFrame()
{
    //Function: process the newly added keyframe
    //compute its BoW & feature vectors & save the normal, depths and descriptors
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        mlNewKeyFrames.pop_front();
    }

    // Compute Bags of Words structures
    if (mpCurrentKeyFrame->isLoadedFromDisk == false)
        mpCurrentKeyFrame->ComputeBoW();
    else
        mpCurrentKeyFrame->ReconstructBoW();
    // Associate MapPoints to the new keyframe and update normal and descriptor
    const vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(size_t i=0; i<vpMapPointMatches.size(); i++)
    {
        MapPoint* pMP = vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                // Is the keyframe loaded from disk
                if (mpCurrentKeyFrame->isLoadedFromDisk == false)
                {
                    if (!pMP->IsInKeyFrame(mpCurrentKeyFrame)) {
                        pMP->AddObservation(mpCurrentKeyFrame, i);
                        pMP->UpdateNormalAndDepth();
                        pMP->ComputeDistinctiveDescriptors();
                    } else // this can only happen for new stereo points inserted by the Tracking
                    {
                        mlpRecentAddedMapPoints.push_back(pMP);
                    }
                }

                else
                {
                    if (!pMP->IsInKeyFrame(mpCurrentKeyFrame))
                    {
//                        cout << "Found in keyframe" << endl;
                        pMP->nObs++;
                        pMP->UpdateNormalAndDepth();
                        pMP->ComputeDistinctiveDescriptors();
                    } else // this can only happen for new stereo points inserted by the Tracking
                    {
//                        cout << "Could not find it in the keyframe" << endl;
                        mlpRecentAddedMapPoints.push_back(pMP);
                    }

                }

            }
        }
    }    

    // Update links in the Covisibility Graph
    mpCurrentKeyFrame->UpdateConnections();

    // Insert Keyframe in Map
    mpMap->AddKeyFrame(mpCurrentKeyFrame);

}

void LocalMapping::MapPointCulling()
{
    // Check Recent Added MapPoints
    list<MapPoint*>::iterator lit = mlpRecentAddedMapPoints.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;


    int nThObs;
    if(mbMonocular)
        nThObs = 2;
    else
        nThObs = 3;
    const int cnThObs = nThObs;

    while(lit!=mlpRecentAddedMapPoints.end())
    {
        MapPoint* pMP = *lit;
        if(pMP->isBad())
        {
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(pMP->GetFoundRatio()<0.25f )
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=2 && pMP->Observations()<=cnThObs)
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
       }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=3) {
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else
            lit++;
    }
}

void LocalMapping::CreateNewMapPoints()
{
    // Retrieve neighbor keyframes in covisibility graph
    int nn = 10;
    if(mbMonocular)
        nn=20;

    //Get nn closest neighboring keyframes
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

    ORBmatcher matcher(0.6, true, false);

    cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotation();
    cv::Mat Rwc1 = Rcw1.t();
    cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslation();
    cv::Mat Tcw1(3,4,CV_32F);
    Rcw1.copyTo(Tcw1.colRange(0,3));
    tcw1.copyTo(Tcw1.col(3));
    cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();

    const float &fx1 = mpCurrentKeyFrame->fx;
    const float &fy1 = mpCurrentKeyFrame->fy;
    const float &cx1 = mpCurrentKeyFrame->cx;
    const float &cy1 = mpCurrentKeyFrame->cy;
    const float &invfx1 = mpCurrentKeyFrame->invfx;
    const float &invfy1 = mpCurrentKeyFrame->invfy;

    const float ratioFactor = 1.5f*mpCurrentKeyFrame->mfScaleFactor;

    static int differentLabels = 0;
    static int sameLabels = 0;

    int nnew=0;

    // Search matches with epipolar restriction and triangulate
    int loadedFromDisk = 0;
    for(size_t i=0; i<vpNeighKFs.size(); i++)
    {
        //If the neighboring keyframe is picked from disk, we cannot do any matching because we have throwed away their keypoints
        if (vpNeighKFs[i]->isLoadedFromDisk == true) {
            loadedFromDisk++;
            continue;
        }

        if(i>0 && CheckNewKeyFrames())
            return;

        KeyFrame* pKF2 = vpNeighKFs[i];

        // Check first that baseline is not too short
        cv::Mat Ow2 = pKF2->GetCameraCenter();
        cv::Mat vBaseline = Ow2-Ow1;
        const float baseline = cv::norm(vBaseline);

        if(!mbMonocular)
        {
            if(baseline<pKF2->mb)
            continue;
        }
        else
        {
            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
            const float ratioBaselineDepth = baseline/medianDepthKF2;

            if(ratioBaselineDepth<0.01)
                continue;
        }

        // Compute Fundamental Matrix
        cv::Mat F12 = ComputeF12(mpCurrentKeyFrame,pKF2);

        // Search matches that fullfil epipolar constraint
        vector<pair<size_t,size_t> > vMatchedIndices;
        //Match only UN-MATCHED KEY POINTS, and store the matched indicies in vMatched Indicies!!!
        matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,F12,vMatchedIndices,false);

        cv::Mat Rcw2 = pKF2->GetRotation();
        cv::Mat Rwc2 = Rcw2.t();
        cv::Mat tcw2 = pKF2->GetTranslation();
        cv::Mat Tcw2(3,4,CV_32F);
        Rcw2.copyTo(Tcw2.colRange(0,3));
        tcw2.copyTo(Tcw2.col(3));


        const float &fx2 = pKF2->fx;
        const float &fy2 = pKF2->fy;
        const float &cx2 = pKF2->cx;
        const float &cy2 = pKF2->cy;
        const float &invfx2 = pKF2->invfx;
        const float &invfy2 = pKF2->invfy;


        // Triangulate each match
        const int nmatches = vMatchedIndices.size();
        for(int ikp=0; ikp<nmatches; ikp++)
        {
            const int &idx1 = vMatchedIndices[ikp].first;
            const int &idx2 = vMatchedIndices[ikp].second;

            const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKeysUn[idx1];
            const float kp1_ur=mpCurrentKeyFrame->mvuRight[idx1];
            bool bStereo1 = kp1_ur>=0;

            const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
            const float kp2_ur = pKF2->mvuRight[idx2];
            bool bStereo2 = kp2_ur>=0;

            // checking to see if the labels are the same
            const unsigned char kp1Label = mpCurrentKeyFrame->mvLabels[idx1];
            const unsigned char kp2Label = pKF2->mvLabels[idx2];

//            //TODO: skipping dynamic map-points
            if (mIsDynamicObjectRemoval == true) {
                if (SemanticSegmentor::IsDynamicObject(kp1Label) || SemanticSegmentor::IsDynamicObject(kp2Label))
                    continue;
            }

            //TODO: Might be a research quesion i.e., how to figure out what label to assign to a map-point if there are multiple labels for it
            if ( kp1Label != kp2Label )
            {
                differentLabels++;

//                cout << "mpCurrentKF [" << mpCurrentKeyFrame->GetmnId() << "] = " << SemanticSegmentor::GetObjectName((unsigned int)kp1Label) << ", pKF2 [" << pKF2->GetmnId() << "] = " << SemanticSegmentor::GetObjectName((unsigned int)kp2Label) << endl;
            }
            else
                sameLabels++;

            // Check parallax between rays
            cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0);
            cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0);

            cv::Mat ray1 = Rwc1*xn1;
            cv::Mat ray2 = Rwc2*xn2;
            const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

            float cosParallaxStereo = cosParallaxRays+1;
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;

            if(bStereo1)
                cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth[idx1]));
            else if(bStereo2)
                cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth[idx2]));

            cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);

            cv::Mat x3D;
            if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<0.9998))
            {
                // Linear Triangulation Method
                cv::Mat A(4,4,CV_32F);
                A.row(0) = xn1.at<float>(0)*Tcw1.row(2)-Tcw1.row(0);
                A.row(1) = xn1.at<float>(1)*Tcw1.row(2)-Tcw1.row(1);
                A.row(2) = xn2.at<float>(0)*Tcw2.row(2)-Tcw2.row(0);
                A.row(3) = xn2.at<float>(1)*Tcw2.row(2)-Tcw2.row(1);

                cv::Mat w,u,vt;
                cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

                x3D = vt.row(3).t();

                if(x3D.at<float>(3)==0)
                    continue;

                // Euclidean coordinates
                x3D = x3D.rowRange(0,3)/x3D.at<float>(3);

            }
            else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
            {
                x3D = mpCurrentKeyFrame->UnprojectStereo(idx1);                
            }
            else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
            {
                x3D = pKF2->UnprojectStereo(idx2);
            }
            else
                continue; //No stereo and very low parallax

            cv::Mat x3Dt = x3D.t();

            //Check triangulation in front of cameras
            float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
            if(z1<=0)
                continue;

            float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
            if(z2<=0)
                continue;

            //Check reprojection error in first keyframe
            const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
            const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);
            const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
            const float invz1 = 1.0/z1;

            if(!bStereo1)
            {
                float u1 = fx1*x1*invz1+cx1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
                    continue;
            }
            else
            {
                float u1 = fx1*x1*invz1+cx1;
                float u1_r = u1 - mpCurrentKeyFrame->mbf*invz1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                float errX1_r = u1_r - kp1_ur;
                if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8*sigmaSquare1)
                    continue;
            }

            //Check reprojection error in second keyframe
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
            const float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);
            const float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
            const float invz2 = 1.0/z2;
            if(!bStereo2)
            {
                float u2 = fx2*x2*invz2+cx2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                    continue;
            }
            else
            {
                float u2 = fx2*x2*invz2+cx2;
                float u2_r = u2 - mpCurrentKeyFrame->mbf*invz2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                float errX2_r = u2_r - kp2_ur;
                if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
                    continue;
            }

            //Check scale consistency
            cv::Mat normal1 = x3D-Ow1;
            float dist1 = cv::norm(normal1);

            cv::Mat normal2 = x3D-Ow2;
            float dist2 = cv::norm(normal2);

            if(dist1==0 || dist2==0)
                continue;

            const float ratioDist = dist2/dist1;
            const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];

            /*if(fabs(ratioDist-ratioOctave)>ratioFactor)
                continue;*/
            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                continue;

            // Triangulation is succesfull
            MapPoint* pMP = new MapPoint(x3D, kp1Label, mpCurrentKeyFrame,mpMap);
            pMP->AppendLabel (kp2Label);

            pMP->AddObservation(mpCurrentKeyFrame,idx1);            
            pMP->AddObservation(pKF2,idx2);

            mpCurrentKeyFrame->AddMapPoint(pMP,idx1);
            pKF2->AddMapPoint(pMP,idx2);

            pMP->ComputeDistinctiveDescriptors();

            pMP->UpdateNormalAndDepth();

            mpMap->AddMapPoint(pMP);
            mlpRecentAddedMapPoints.push_back(pMP);


            nnew++;
        }
    }

//    cout << "New map-points = " << nnew << endl;
//    cout << "Same labels = " << sameLabels << endl;
//    cout << "Different labels = " << differentLabels << endl;

}

void LocalMapping::SearchInNeighbors()
{
    // Retrieve neighbor keyframes
    int nn = 10;
    if(mbMonocular)
        nn=20;
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
    //Get the 10 closest key frames to the current one in the co-visibility graph
    vector<KeyFrame*> vpTargetKFs;
    for(vector<KeyFrame*>::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        if(pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
            continue;
        vpTargetKFs.push_back(pKFi);//Only push valid keyframes in vpTargetKFs
        pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;

        // Extend to some second neighbors
        const vector<KeyFrame*> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
        for(vector<KeyFrame*>::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
        {
            KeyFrame* pKFi2 = *vit2;
            if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF==mpCurrentKeyFrame->mnId || pKFi2->mnId==mpCurrentKeyFrame->mnId)
                continue;
            vpTargetKFs.push_back(pKFi2);
        }
    }

    //In vpTargetKFs we have (nn + 1) * 5 closest keyframes to the current one, including nn immediate neighbors and 5 second neighbors of each nn neighbors


    // Search matches by projection from current KF in target KFs
    // Project
    ORBmatcher matcher;
    vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(vector<KeyFrame*>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;


        //Making changes-KPs
        //If the keyframe is loaded from disk, continue because we have not stored its keypoints
        if (pKFi->isLoadedFromDisk == true) {
            continue;
        }
        else
        {
            //If the keyframe is from the current session, do a FUSE operation since we have its key points
            matcher.Fuse(pKFi,vpMapPointMatches);
        }

    }

    // Search matches by projection from target KFs in current KF
    vector<MapPoint*> vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

    for(vector<KeyFrame*>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
    {
        //Iterate through all other keyframes
        KeyFrame* pKFi = *vitKF;

        vector<MapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches();
        //Get the map point matches for all other keyframes

        for(vector<MapPoint*>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
        {
            MapPoint* pMP = *vitMP;
            if(!pMP)
                continue;
            if(pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
                continue;
            pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
            vpFuseCandidates.push_back(pMP);
        }
    }

//    cout << "Now going to fuse mpCurrentKeyFrame with vpFuseCandidates" << endl;
    matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates);


    // Update points
    vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
    {
        MapPoint* pMP=vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                pMP->ComputeDistinctiveDescriptors();
                pMP->UpdateNormalAndDepth();
            }
        }
    }

    // Update connections in covisibility graph
    mpCurrentKeyFrame->UpdateConnections();
}

cv::Mat LocalMapping::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)
{
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    cv::Mat R12 = R1w*R2w.t();
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

    cv::Mat t12x = SkewSymmetricMatrix(t12);

    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;


    return K1.t().inv()*t12x*R12*K2.inv();
}

void LocalMapping::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
    unique_lock<mutex> lock2(mMutexNewKFs);
    mbAbortBA = true;
}

bool LocalMapping::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
//        cout << "Local Mapping STOP" << endl;
        return true;
    }

    return false;
}

bool LocalMapping::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool LocalMapping::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

void LocalMapping::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    if(mbFinished)
        return;
    mbStopped = false;
    mbStopRequested = false;
    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
        delete *lit;
    mlNewKeyFrames.clear();

//    cout << "Local Mapping RELEASE" << endl;
}

bool LocalMapping::AcceptKeyFrames()
{
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

bool LocalMapping::SetNotStop(bool flag)
{
    unique_lock<mutex> lock(mMutexStop);

    if(flag && mbStopped)
        return false;

    mbNotStop = flag;

    return true;
}

void LocalMapping::InterruptBA()
{
    mbAbortBA = true;
}

void LocalMapping::KeyFrameCulling()
{
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points

    //Get all the co-visible keyframes for the current keyframe
    vector<KeyFrame*> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    //Iterate through all the keyframes
    for(vector<KeyFrame*>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        KeyFrame* pKF = *vit;
        if(pKF->mnId==0)
            continue;

        //Get all map point matches for the current keyframe
        const vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();

        int nObs = 3;
        const int thObs=nObs;
        int nRedundantObservations=0;
        int nMPs=0;
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    if(!mbMonocular)
                    {
                        float depth;
                        if (pKF->isLoadedFromDisk == true)
                            depth = pMP->mpObservation.find(pKF)->second.depth;
                        else
                            depth = pKF->mvDepth[i];

                        const float mpDepth = depth;
                        if( depth > pKF->mThDepth || depth < 0)
                            continue;
                    }

                    nMPs++;
                    //see if the number of observations for the map point are more than the threshold (3)
                    if(pMP->Observations()>thObs)
                    {
                        int scale;
                        if (pKF->isLoadedFromDisk == true)
                            scale = pMP->mpObservation.find(pKF)->second.Keypoint.octave;
                        else
                            scale = pKF->mvKeysUn[i].octave;

                        const int &scaleLevel = scale;

                        //Get all the observations for the map point whose observations are beyond the threshold
                        const map<KeyFrame*, size_t> observations = pMP->GetObservations();
                        int nObs=0;

                        //iterate through all of its observations
                        for(map<KeyFrame*, size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {
                            KeyFrame* pKFi = mit->first;
                            if(pKFi==pKF)
                                continue;

                            int scalei;
                            if (pKFi->isLoadedFromDisk == true)
                                scalei = pMP->mpObservation.find(pKFi)->second.Keypoint.octave;
                            else
                                scalei = pKFi->mvKeysUn[mit->second].octave;

                            const int &scaleLeveli = scalei;

                            if(scaleLeveli<=scaleLevel+1)
                            {
                                nObs++;
                                if(nObs>=thObs)
                                    break;
                            }
                        }
                        if(nObs>=thObs)
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }
        }  

        if(nRedundantObservations>0.9*nMPs)
            pKF->SetBadFlag();
    }
}

cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2),               0,-v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),              0);
}

void LocalMapping::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
                break;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(3000));
    }
}

void LocalMapping::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        mlNewKeyFrames.clear();
        mlpRecentAddedMapPoints.clear();
        mbResetRequested=false;
    }
}

void LocalMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LocalMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LocalMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
}

bool LocalMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

} //namespace ORB_SLAM
