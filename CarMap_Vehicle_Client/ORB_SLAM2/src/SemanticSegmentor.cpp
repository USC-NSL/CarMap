//
// Created by on 12/30/18.
//
#include "SemanticSegmentor.h"

namespace ORB_SLAM2
{
    namespace SemanticSegmentor
    {
        // functions definitions
        cv::Scalar GetColorCode(unsigned int index) {
            return colorCodes[index];
        }

        const std::string GetObjectName(unsigned int index) {
            if (index < SEMANTIC_SEGMENTOR_NUMBER_OF_OBJECTS - 1)
            return objectNames[index];
            else
                return objectNames[UNDETECTED_CODE];
        }

        bool IsDynamicObject (unsigned int index)
        {
            if ( index == PERSON_CODE || index == RIDER_CODE || index == CAR_CODE || index == TRUCK_CODE || index == BUS_CODE || index == TRAIN_CODE || index == MOTORCYCLE_CODE || index == BICYCLE_CODE )
                return true;
            else
                return false;
        }

        bool IsStaticObject (unsigned int index)
        {
            return ( ( (!IsDynamicObject(index)) && (index != UNDETECTED_CODE) ) );
        }

        bool IsSemiDynamicObject (unsigned int index)
        {
            if ( index == CAR_CODE || index == TRUCK_CODE || index == BUS_CODE || index == TRAIN_CODE )
                return true;
            else
                return false;
        }

        bool IsCarObjectForCarla (unsigned int index)
        {
            if ( index == CAR_CODE || index == TRUCK_CODE || index == BUS_CODE || index == TRAIN_CODE || index == MOTORCYCLE_CODE)
                return true;
            else
                return false;
        }
    }
}
