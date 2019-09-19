//
// Created by on 1/22/19.
//

#ifndef CROWDSOURCED_HD_MAP_KDTREE_H
#define CROWDSOURCED_HD_MAP_KDTREE_H

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <MapPoint.h>

namespace ORB_SLAM2 {
    class KDTree
    {
    private:
        // Function members
        void ConstructKDTree ();
    protected:
    public:
        // Function members
        // Constructor
        KDTree ();
        KDTree(vector <ORB_SLAM2::MapPoint *> inputMps);
        KDTree(vector <ORB_SLAM2::KeyFrame *> inputKfs);
        // Search
        vector <MapPoint*> GetNeighbors (float* centralPoint, float radius);
        vector <KeyFrame*> GetNeighbors (float* centralPoint, float radius, bool isKf);

    public:
        // Data members
        int NumberOfPoints;
        pcl::PointCloud <pcl::PointXYZ> cloud;
        pcl::KdTreeFLANN <pcl::PointXYZ> kdtree;
        vector <MapPoint*> allMps;
        vector <KeyFrame*> allKfs;

    };
}

#endif //CROWDSOURCED_HD_MAP_KD_TREE_H
