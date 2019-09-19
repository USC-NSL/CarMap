//
// Created by  on 12/24/18.
//

#ifndef CROWDSOURCED_HD_MAP_SEMANTICSEGMENTOR_H
#define CROWDSOURCED_HD_MAP_SEMANTICSEGMENTOR_H

#include<opencv2/core/core.hpp>
namespace ORB_SLAM2 {
    namespace SemanticSegmentor {

#define SEMANTIC_SEGMENTOR_NUMBER_OF_OBJECTS 21
#define ROAD_CODE 0
#define SIDEWALK_CODE 1
#define BUILDING_CODE 2
#define WALL_CODE 3
#define FENCE_CODE 4
#define POLE_CODE 5
#define TRAFFIC_LIGHT_CODE 6
#define TRAFFIC_SIGN_CODE 7
#define VEGETATION_CODE 8
#define TERRAIN_CODE 9
#define SKY_CODE 10
#define PERSON_CODE 11
#define RIDER_CODE 12
#define CAR_CODE 13
#define TRUCK_CODE 14
#define BUS_CODE 15
#define TRAIN_CODE 16
#define MOTORCYCLE_CODE 17
#define BICYCLE_CODE 18
#define LICENSE_PLATE_CODE 19
#define UNDETECTED_CODE 20

        bool static dynamicObjectRemoval = true;

        const cv::Scalar colorCodes[SEMANTIC_SEGMENTOR_NUMBER_OF_OBJECTS] = {
                cv::Scalar(128, 64, 128), //0 road
                cv::Scalar(244, 35, 232), //1 sidewalk
                cv::Scalar(70, 70, 70), //2 building
                cv::Scalar(102, 102, 156), //3 wall
                cv::Scalar(190, 153, 153), //4 fence
                cv::Scalar(153, 153, 153), //5 pole
                cv::Scalar(250, 170, 30), //6 traffic light
                cv::Scalar(220, 220, 0), //7 traffic sign
                cv::Scalar(107, 142, 35), //8 vegetation
                cv::Scalar(152, 251, 152), //9 terrain
                cv::Scalar(70, 130, 180), //10 sky
                cv::Scalar(220, 20, 60), //11 person
                cv::Scalar(255, 0, 0), //12 rider
                cv::Scalar(0, 0, 142), //13 car
                cv::Scalar(0, 0, 70), //14 truck
                cv::Scalar(0, 60, 100), //15 bus
                cv::Scalar(0, 80, 100), //16 train
                cv::Scalar(0, 0, 230), //17 motorcyle
                cv::Scalar(119, 11, 32), //18 bicycle
                cv::Scalar(0, 0, 142), //19 license plate
                cv::Scalar(0, 0, 0) //20 undetected
        };


        const std::string objectNames[SEMANTIC_SEGMENTOR_NUMBER_OF_OBJECTS] = {
                "Road",
                "Sidewalk",
                "Building",
                "Wall",
                "Fence",
                "Pole",
                "Traffic light",
                "Traffic sign",
                "Vegetation",
                "Terrain",
                "Sky",
                "Person",
                "Rider",
                "Car",
                "Truck",
                "Bus",
                "Train",
                "Motorcycle",
                "Bicycle",
                "License plate",
                "Untrained"
        };


        cv::Scalar GetColorCode(unsigned int index);

        const std::string GetObjectName(unsigned int index);

        bool IsDynamicObject (unsigned int index);

        bool IsSemiDynamicObject (unsigned int index);

        bool IsCarObjectForCarla (unsigned int index);

        bool IsStaticObject (unsigned int index);

    };
}


#endif //CROWDSOURCED_HD_MAP_SEMANTICSEGMENTOR_H
