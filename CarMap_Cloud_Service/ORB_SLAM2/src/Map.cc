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

#include "Map.h"

#include<mutex>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

namespace ORB_SLAM2 {

    Map::Map() : mnMaxKFid(0), mnBigChangeIdx(0) {
    }

    void Map::AddKeyFrame(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexMap);
        mspKeyFrames.insert(pKF);
        if (pKF->mnId > mnMaxKFid)
            mnMaxKFid = pKF->mnId;
    }

    void Map::AddMapPoint(MapPoint *pMP) {
        unique_lock<mutex> lock(mMutexMap);
        mspMapPoints.insert(pMP);
    }

    void Map::EraseMapPoint(MapPoint *pMP) {
        unique_lock<mutex> lock(mMutexMap);
        mspMapPoints.erase(pMP);

        // TODO: This only erase the pointer.
        // Delete the MapPoint
    }

    void Map::EraseKeyFrame(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexMap);
        mspKeyFrames.erase(pKF);

        // TODO: This only erase the pointer.
        // Delete the MapPoint
    }

    void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs) {
//    cout << "Setting reference map points" << endl;
        unique_lock<mutex> lock(mMutexMap);
        mvpReferenceMapPoints = vpMPs;
    }

    void Map::InformNewBigChange() {
        unique_lock<mutex> lock(mMutexMap);
        mnBigChangeIdx++;
    }

    int Map::GetLastBigChangeIdx() {
        unique_lock<mutex> lock(mMutexMap);
        return mnBigChangeIdx;
    }

    vector<KeyFrame *> Map::GetAllKeyFrames() {
        unique_lock<mutex> lock(mMutexMap);
        return vector<KeyFrame *>(mspKeyFrames.begin(), mspKeyFrames.end());
    }

    vector<MapPoint *> Map::GetAllMapPoints() {
        unique_lock<mutex> lock(mMutexMap);
        return vector<MapPoint *>(mspMapPoints.begin(), mspMapPoints.end());
    }

    long unsigned int Map::MapPointsInMap() {
        unique_lock<mutex> lock(mMutexMap);
        return mspMapPoints.size();
    }

    long unsigned int Map::KeyFramesInMap() {
        unique_lock<mutex> lock(mMutexMap);
        return mspKeyFrames.size();
    }

    void Map::ConstructKDTree ()
    {
        vector <MapPoint*> allMps = GetAllMapPoints();
        this->kdtree = new KDTree (allMps); // create a new kd tree structure

        vector <KeyFrame*> allKfs = GetAllKeyFrames();
        this->kfKDtree = new KDTree (allKfs);
    }

    vector <MapPoint*> Map::GetKDTreeNeighbors (float* searchSpot, float searchRadius)
    {
        return this->kdtree->GetNeighbors(searchSpot, searchRadius);
    }

//    void Map::SavePointClouds(vector<ORB_SLAM2::MapPoint*> inputMps)
//    {
//        cout << "Saving point clouds" << endl;
//        ObjectCluster objCluster = ObjectCluster (inputMps);
//
//        objCluster.ConstructKDTree();
//        objCluster.ExtractClusters();
//
//        cout << "Saved point clouds" << endl;
//    }

    void Map::PrintMapPointLabelStatistics(unsigned int *labelCounters) {

        cout << "****************************************" << endl;
        cout << "************* Label statistics **********" << endl;
        vector <MapPoint*> allMps = GetAllMapPoints();

        for (MapPoint * mp : allMps)
        {
            if ( mp->GetSemanticLabel() > SEMANTIC_SEGMENTOR_NUMBER_OF_OBJECTS - 1 )
                labelCounters [SEMANTIC_SEGMENTOR_NUMBER_OF_OBJECTS - 1]++;
            else
                labelCounters [mp->GetSemanticLabel()]++;
        }

//        for ( unsigned int counter = 0; counter < SEMANTIC_SEGMENTOR_NUMBER_OF_OBJECTS; counter++ )
//        {
//            cout << labelCounters [counter] << endl;
//        }
//
//
//
        for ( unsigned int counter = 0; counter < SEMANTIC_SEGMENTOR_NUMBER_OF_OBJECTS; counter++ )
        {
            string label = SemanticSegmentor::GetObjectName(counter);
            cout << SemanticSegmentor::GetObjectName(counter) << " : " << ( ( labelCounters [counter] * 100.0 ) / allMps.size() )<< endl;
        }

        cout << "**************************************" << endl;
        return ;


    }

    void Map::PrintResultsFromDiffOperation ()
    {
        vector <MapPoint*> allMps = GetAllMapPoints();

        unsigned int missedObjects [SEMANTIC_SEGMENTOR_NUMBER_OF_OBJECTS] = {0};

        unsigned int numberOfDetections = 0;
        unsigned int numberOfMisdetections = 0;
        unsigned int firstTimeDetections = 0;

        unsigned int detectedObjects [SEMANTIC_SEGMENTOR_NUMBER_OF_OBJECTS] = {0};

        vector <ORB_SLAM2::MapPoint*> carMPs;


        for (MapPoint * mp : allMps)
        {
            if ( mp->GetFromPreviousSession() == true ) // if the map-point is from the previous map
            {
                if ( mp->GetNumberOfOccurencesInNewSession() == 0 ) // if the map-point has not been seen in this map
                {
                    numberOfMisdetections++;
                    missedObjects [mp->GetSemanticLabel()]++;
                }
                else
                    numberOfDetections++;
            }
            else
            {
                if (mp != NULL)
                    if (mp->isBad() == false)
                    {
                        detectedObjects[mp->GetSemanticLabel()]++;
                        firstTimeDetections++;
                        if (mp->GetSemanticLabel() == CAR_CODE)
                            carMPs.push_back(mp);
                    }
            }
        }

        for (MapPoint *mp : allMps)
        {

        }

        cout << "Number of map-points detected again = " << numberOfDetections << " / " << numberOfDetections + numberOfMisdetections << " = " << ( numberOfDetections * 100.0 ) / ( numberOfDetections + numberOfMisdetections ) << " %" << endl;
        cout << "Number of map-points not detected   = " << numberOfMisdetections << " / " << numberOfDetections + numberOfMisdetections << " = " << ( numberOfMisdetections * 100.0 ) / ( numberOfDetections + numberOfMisdetections ) << " %" << endl;
        cout << "Number of map-points detected for the first time = " << firstTimeDetections << " / " << allMps.size() << " = " << ( firstTimeDetections * 100.0 ) / ( allMps.size() ) << " %" << endl;


        cout << "*********************************************************" << endl;
        cout << "*********************************************************" << endl;
        cout << "Printing all the semi-dynamic objects that were not detected again" << endl;
        cout << "*********************************************************" << endl;
        cout << "*********************************************************" << endl;



        for ( unsigned int counter = 0; counter < SEMANTIC_SEGMENTOR_NUMBER_OF_OBJECTS; counter++ )
        {
            if (SemanticSegmentor::IsSemiDynamicObject(counter) == true)
                cout << SemanticSegmentor::GetObjectName (counter) << " : " <<  ( missedObjects [counter] * 100.0 ) / ( numberOfMisdetections ) << endl;
        }


        cout << "*********************************************************" << endl;
        cout << "*********************************************************" << endl;
        cout << "Printing all the semi-dynamic objects that were detected for the first time" << endl;
        cout << "*********************************************************" << endl;
        cout << "*********************************************************" << endl;


        for ( unsigned int counter = 0; counter < SEMANTIC_SEGMENTOR_NUMBER_OF_OBJECTS; counter++ )
        {
            if (SemanticSegmentor::IsSemiDynamicObject(counter) == true)
                cout << SemanticSegmentor::GetObjectName (counter) << " : " <<  ( detectedObjects [counter] * 100.0 ) / ( firstTimeDetections ) << endl;
        }

        cout << "Performing extraction now" << endl;

//        SavePointClouds(carMPs);


    }


    vector<MapPoint *> Map::GetReferenceMapPoints() {
//    cout << "Accessing reference map points" << endl;
        unique_lock<mutex> lock(mMutexMap);
        return mvpReferenceMapPoints;
    }


    long unsigned int Map::GetMaxKFid() {
        unique_lock<mutex> lock(mMutexMap);
        return mnMaxKFid;
    }

    void Map::clear() {
        for (set<MapPoint *>::iterator sit = mspMapPoints.begin(), send = mspMapPoints.end(); sit != send; sit++)
            delete *sit;

        for (set<KeyFrame *>::iterator sit = mspKeyFrames.begin(), send = mspKeyFrames.end(); sit != send; sit++)
            delete *sit;

        mspMapPoints.clear();
        mspKeyFrames.clear();
        mnMaxKFid = 0;
        mvpReferenceMapPoints.clear();
        mvpKeyFrameOrigins.clear();
    }

    template<class Archive>
    void Map::serialize(Archive &ar, const unsigned int version) {
        // don't save mutex
        bool DEBUG_MODE = false;
//    if (DEBUG_MODE)
//    {
        string mspMP, mvpKFO, mspKF, mvpRMP;
        mspMP = "\nAll map points\n";
        mvpKFO = "\nKeyframe origins\n";
        mspKF = "\nAll keyframes\n";
        mvpRMP = "\nReference map points\n";
//    }

        if (DEBUG_MODE)
            ar & mspMP;
        ar & mspMapPoints;
        if (DEBUG_MODE)
            ar & mvpKFO;
        ar & mvpKeyFrameOrigins;
        if (DEBUG_MODE)
            ar & mspKF;
        ar & mspKeyFrames;
//    if (DEBUG_MODE)
//        ar & mvpRMP;
//    ar & mvpReferenceMapPoints;
        //TODO: Uncomment these
//    ar & mnMaxKFid & mnBigChangeIdx;


//    ar & const_cast<float &>(fx) & const_cast<float &>(fy) & const_cast<float &>(cx) & const_cast<float &>(cy);
//    ar & const_cast<float &>(invfx) & const_cast<float &>(invfy) & const_cast<float &>(mbf);
//    ar & const_cast<float &>(mb) & const_cast<float &>(mThDepth);
    }



    template void Map::serialize(boost::archive::text_oarchive &, const unsigned int);

    template void Map::serialize(boost::archive::text_iarchive &, const unsigned int);

    template void Map::serialize(boost::archive::binary_iarchive &, const unsigned int);

    template void Map::serialize(boost::archive::binary_oarchive &, const unsigned int);

} //namespace ORB_SLAM
