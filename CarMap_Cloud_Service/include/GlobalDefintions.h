//
// Created by on 2/22/18.
//

#ifndef CROWDSOURCED_HD_MAP_GLOBALDEFINTIONS_H
#define CROWDSOURCED_HD_MAP_GLOBALDEFINTIONS_H

const bool ON = true;
const bool OFF = false;

const std::string COMPRESSION_LEVEL = "CarMap_";

const bool DEBUG_MODE = ON;
const bool GET_STATIC_FEATURES = OFF;
const bool SPATIAL_FEATURE_MODE = OFF;
const bool ORB_SLAM_MODE = ON;
const bool ZED_SLAM_MODE = OFF;
const bool EVALUATION_MODE = OFF;
const bool FEATURE_DEBUG_MODE = ON;
const bool DYNAMIC_OBJECTS_REMOVAL = OFF;

enum OPERATION_MODE
{
    SAVE_MAP,//Init mode without previous map
    LOAD_MAP,//Load map and build on top of that
    UPDATE_MAP//Load map, build on top of it and save the map
};

OPERATION_MODE QS_State = SAVE_MAP;
#endif //CROWDSOURCED_HD_MAP_GLOBALDEFINTIONS_H
