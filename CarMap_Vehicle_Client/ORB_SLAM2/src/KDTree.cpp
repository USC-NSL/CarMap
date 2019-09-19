//
// Created by
// on 1/22/19.
//

#include <KDTree.h>

namespace ORB_SLAM2 {

    // constructor
    KDTree::KDTree(vector <ORB_SLAM2::MapPoint *> inputMps)
    {
        this->NumberOfPoints = inputMps.size();
        this->allMps = inputMps;
        // construct the point cloud
        this->cloud.width = this->NumberOfPoints;
        this->cloud.height = 1;
        this->cloud.resize(this->cloud.width * this->cloud.height);
        int cloudCounter = 0;
        for ( ORB_SLAM2::MapPoint* mp : inputMps )
        {
            this->cloud.points [cloudCounter].x = mp->GetWorldPos().at<float> (0,0);
            this->cloud.points [cloudCounter].y = mp->GetWorldPos().at<float> (1,0);
            this->cloud.points [cloudCounter].z = mp->GetWorldPos().at<float> (2,0);
            cloudCounter++;
        }
        // Constructing the KD Tree structure
        ConstructKDTree();
        cout << "Constructed mp kd-tree" << endl;
    }

    KDTree::KDTree (vector <ORB_SLAM2::KeyFrame*> input_keyframes)
    {
        this->NumberOfPoints = input_keyframes.size();
        this->allKfs = input_keyframes;
        // construct the point cloud
        this->cloud.width = this->NumberOfPoints;
        this->cloud.height = 1;
        this->cloud.resize(this->cloud.width * this->cloud.height);
        int cloudCounter = 0;
        for ( ORB_SLAM2::KeyFrame* kf : input_keyframes )
        {
            this->cloud.points [cloudCounter].x = kf->GetPose().at<float> (0,3);
            this->cloud.points [cloudCounter].y = kf->GetPose().at<float> (1,3);
            this->cloud.points [cloudCounter].z = kf->GetPose().at<float> (2,3);
            cloudCounter++;
        }
        // Constructing the KD Tree structure
        ConstructKDTree();
        cout << "Constructed keyframe kd-tree" << endl;
    }

    // Construct the KD Tree structure from the point cloud
    void KDTree::ConstructKDTree ()
    {
        pcl::PointCloud <pcl::PointXYZ>::Ptr cloudPtr = (pcl::PointCloud <pcl::PointXYZ>::Ptr) &this->cloud;
        kdtree.setInputCloud (cloudPtr);
    }

    // Print the neighbors for the point centralPoint within radius radius
    vector <MapPoint*> KDTree::GetNeighbors (float* centralPoint, float radius)
    {
        pcl::PointXYZ searchPoint;
        searchPoint.x = centralPoint[0];
        searchPoint.y = centralPoint[1];
        searchPoint.z = centralPoint[2];

        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        std::vector <MapPoint*> neighboringMapPoints;
        if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
        {
            for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i) {
                MapPoint *mp = this->allMps.at(pointIdxRadiusSearch[i]);
                neighboringMapPoints.push_back(mp);
            }
        }
        return neighboringMapPoints;
    }

    // Print the neighbors for the point centralPoint within radius radius
    vector <KeyFrame*> KDTree::GetNeighbors (float* centralPoint, float radius, bool isKF)
    {
        pcl::PointXYZ searchPoint;
        searchPoint.x = centralPoint[0];
        searchPoint.y = centralPoint[1];
        searchPoint.z = centralPoint[2];

        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        std::vector <KeyFrame*> neighboringKeyframes;
        if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
        {
            for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i) {
                KeyFrame *kf = this->allKfs.at(pointIdxRadiusSearch[i]);
                neighboringKeyframes.push_back(kf);
            }
        }
        return neighboringKeyframes;
    }
}