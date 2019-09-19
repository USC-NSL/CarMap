//
// Created by on 7/17/18.
//

#ifndef CROWDSOURCED_HD_MAP_MAPPOINTOBSERVATION_H
#define CROWDSOURCED_HD_MAP_MAPPOINTOBSERVATION_H


#include <boost/serialization/split_member.hpp>

const std::string pathToCameraCalibration = "CamCalib.yaml";

class MapPoint_Observation
{
public:
    //Data members
    cv::KeyPoint Keypoint;//2D coordinates of the  keypoint
    float depth;//Depth of the key point
    float rightCoordinate;//Right coordinate of the keypoint
    cv::Mat descriptor;//Descriptor of the keypoint
    bool loadedFromDisk;
    size_t index;




public:
    //Function definitions

    //Default Constructor
    MapPoint_Observation ()
    {
        Keypoint.pt.x = Keypoint.pt.y = 0;
        rightCoordinate = depth = 0;
        descriptor = 0;
        index = 0;
        loadedFromDisk = false;
    }


    //Parameterized Construction
    MapPoint_Observation (cv::KeyPoint KPval, float depthVal, float rightVal, cv::Mat descriptorVal):
            Keypoint(KPval), depth(depthVal), rightCoordinate(rightVal), descriptor(descriptorVal)
    {};

//    MapPoint_Observation::MapPoint_Observation ():
//    depth(0), rightCoordinate(0), index(0)
//    {
//        Keypoint.pt.x = 0;
//        Keypoint.pt.y = 0;
//        descriptor.
//    }

private:

    //Save function
    friend class boost::serialization::access;
    template <class Archive>
    void save (Archive & ar, const unsigned int version) const
    {
        ar << Keypoint;
        ar << depth;
        ar << rightCoordinate;
        ar << index;
//        ar << descriptor;
    }


    //Load function
//    friend class boost::serialization::access;
    template <class Archive>
    void load (Archive & ar, const unsigned int version)
    {
        ar >> Keypoint;
        ar >> depth;
        ar >> rightCoordinate;
        ar >> index;
//        ar >> descriptor;
        Init_MpObs ();
    }
    BOOST_SERIALIZATION_SPLIT_MEMBER()

public:
    //Note that the load function was used
    void Init_MpObs ()
    {
//        cout << "MP loaded from disk" << endl;
        loadedFromDisk = true;
    }

    cv::Mat GetDescriptor ()
    {
        if (isLoadedFromDisk() == false)
            return descriptor;
        else
        {
            cerr << "This is not going to end well" << endl;
            return descriptor;
        }
    }

    cv::KeyPoint GetKeypoint ()
    {
        return Keypoint;
    }

    float GetDepth ()
    {
        return depth;
    }

    float GetRightCoordinate ()
    {
        return rightCoordinate;
    }

    bool isLoadedFromDisk ()
    {
        return loadedFromDisk;
    }


};
#endif //CROWDSOURCED_HD_MAP_MAPPOINTOBSERVATION_H
