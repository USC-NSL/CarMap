//
// Created by on 9/6/17.
//
#include <ZEDInitialization.h>
#include <Eigen/LU>
#include <GlobalDefintions.h>

/*
using namespace sl;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// ZED CAMERA FUNCTIONS ////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//ZED Camera Initialization
int initializeZED (Camera *ptrToZEDCamera, InitParameters *initParameters, bool liveMode = false, sl::String pathToInputFile = "")
{
    //InitParameters for Initializing ZED Camera
    initParameters->camera_resolution = RESOLUTION_HD720;
    initParameters->camera_fps = 30;
    initParameters->depth_mode = DEPTH_MODE_PERFORMANCE;
    initParameters->coordinate_units = UNIT_METER;
    initParameters->coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP;
//    initParameters->camera_image_flip = camera_flipped;


    if (liveMode == false)
        initParameters->svo_input_filename = pathToInputFile;

    ERROR_CODE zedCamStatus = ptrToZEDCamera->open(*initParameters);

    if (zedCamStatus != SUCCESS) {
        cout << "Error in opening ZED Camera " << errorCode2str(zedCamStatus) << endl;
        ptrToZEDCamera->close();
        return -1;
    }
    else {
//        if (!EVALUATION_MODE)
//            cout << "Initialized ZED SDK" << endl;
        return SUCCESS;
    }
}

void saveTrajectoryORBSLAMinSLAMMode (ORB_SLAM2::System * ptrToORBSLAM, const std::string pathToSaveTrajectory)
{
    ptrToORBSLAM->SaveTrajectoryKITTI(pathToSaveTrajectory);
    cout << "Trajectory saved at " << pathToSaveTrajectory << endl;
}


void saveTrajectoryInMonoCamera (ORB_SLAM2::System * ptrToORBSLAM, const std::string pathToSaveTrajectory)
{
    ptrToORBSLAM->SaveTrajectoryTUM(pathToSaveTrajectory);
    cout << "Trajectory saved at " << pathToSaveTrajectory << endl;
}
//ZED SLAM initialization
void initializeZEDSLAM (Camera *ptrToZEDCamera, TrackingParameters *ptrToTrackingParams, const string pathToZEDTrackingOutput, bool reuseMap = false, String pathToMapFile = "")
{
    ptrToTrackingParams->initial_world_transform = sl::Transform::identity();
    ptrToTrackingParams->enable_spatial_memory = true;
    if (reuseMap == true)
        ptrToTrackingParams->area_file_path = pathToMapFile;//give path to map file
    //zedTrackingFile.open (pathToZEDTrackingOutput);


    ERROR_CODE trackingErrorStatus = ptrToZEDCamera->enableTracking(*ptrToTrackingParams);

    if (trackingErrorStatus != SUCCESS)
    {
        std::cout << "Error in initializing ZED SLAM " << errorCode2str(trackingErrorStatus) << endl;
    }
    else
        std::cout << "Initialized ZED SLAM" << endl;

    return;
}
*/

//SL Mat to CV Mat conversion
/*
cv::Mat slMat2cvMat(sl::Mat& input) {
    //convert MAT_TYPE to CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case sl::MAT_TYPE_32F_C1:
            cv_type = CV_32FC1;
            break;
        case sl::MAT_TYPE_32F_C2:
            cv_type = CV_32FC2;
            break;
        case sl::MAT_TYPE_32F_C3:
            cv_type = CV_32FC3;
            break;
        case sl::MAT_TYPE_32F_C4:
            cv_type = CV_32FC4;
            break;
        case sl::MAT_TYPE_8U_C1:
            cv_type = CV_8UC1;
            break;
        case sl::MAT_TYPE_8U_C2:
            cv_type = CV_8UC2;
            break;
        case sl::MAT_TYPE_8U_C3:
            cv_type = CV_8UC3;
            break;
        case sl::MAT_TYPE_8U_C4:
            cv_type = CV_8UC4;
            break;
        default:
            break;
    }

    // cv::Mat data requires a uchar* pointer. Therefore, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    //cv::Mat and sl::Mat will share the same memory pointer
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
}

*/

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// ORB SLAM 2 Interface ////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ORB SLAM2 Functions
ORB_SLAM2::System *initializeORBSlam (string vocabFile, string settingFile, bool visualization, bool saveMap, bool reuseMap, string pathToLoadMapFile, bool runLocalizationMode, bool saveTrajectory, string pathToSaveTrajectory, bool isStereo, const string serverAddr, const int portNumber, bool isUploadMap, string viewerWindowName, bool reconstructMap, bool isStitchMode, bool isDynamicObjectRemoval)
{

    isSaveTrajectory = saveTrajectory;

    if (isStereo == true)
        return new ORB_SLAM2::System (vocabFile, settingFile, ORB_SLAM2::System::STEREO, visualization, saveMap, reuseMap, pathToLoadMapFile, runLocalizationMode, serverAddr, portNumber, isUploadMap, viewerWindowName, reconstructMap, isStitchMode, isDynamicObjectRemoval);
    else
        return new ORB_SLAM2::System (vocabFile, settingFile, ORB_SLAM2::System::MONOCULAR, visualization, saveMap, reuseMap, pathToLoadMapFile, runLocalizationMode, serverAddr, portNumber, isUploadMap, viewerWindowName, reconstructMap, isStitchMode, isDynamicObjectRemoval);
}


// ORB SLAM2 Functions
ORB_SLAM2::System *initializeORBSlam (string vocabFile, string settingFile, bool visualization, bool saveMap, bool reuseMap, string pathToLoadMapFile, bool runLocalizationMode, bool saveTrajectory, string pathToSaveTrajectory, bool isStereo, const string serverAddr, const int portNumber, bool isUploadMap, string viewerWindowName, bool reconstructMap, bool isStitchMode, bool isDynamicObjectRemoval, bool majorityVoting, bool isRobustFeatureSearch)
{

    isSaveTrajectory = saveTrajectory;

    if (isStereo == true)
        return new ORB_SLAM2::System (vocabFile, settingFile, ORB_SLAM2::System::STEREO, visualization, saveMap, reuseMap, pathToLoadMapFile, runLocalizationMode, serverAddr, portNumber, isUploadMap, viewerWindowName, reconstructMap, isStitchMode, isDynamicObjectRemoval, majorityVoting, isRobustFeatureSearch);
    else
        return new ORB_SLAM2::System (vocabFile, settingFile, ORB_SLAM2::System::MONOCULAR, visualization, saveMap, reuseMap, pathToLoadMapFile, runLocalizationMode, serverAddr, portNumber, isUploadMap, viewerWindowName, reconstructMap, isStitchMode, isDynamicObjectRemoval);
}

/*
void runORBSLAMTracking (ORB_SLAM2::System *ptrToORBSLAMSystem, sl::Mat zedLeftView, sl::Mat zedRightView, int timeCounter, bool isStereo, ofstream* featureFile)
{
    cv::Mat leftViewCV, rightViewCV, leftViewCVGray, rightViewCVGray;
    leftViewCV = slMat2cvMat(zedLeftView);
    rightViewCV = slMat2cvMat(zedRightView);

    //Left and right gray scale images
    cv::cvtColor(leftViewCV, leftViewCVGray, CV_BGR2GRAY);
    cv::cvtColor(rightViewCV, rightViewCVGray, CV_BGR2GRAY);

    cv::Mat Tcw;

    if (isStereo == true)
        Tcw = ptrToORBSLAMSystem->TrackStereo(leftViewCV, rightViewCV, timeCounter, featureFile);
    else
        Tcw = ptrToORBSLAMSystem->TrackMonocular(leftViewCV, timeCounter);

    return;
}
*/

void printNumberOfKeyframes (ORB_SLAM2::System *ptrToORBSLAMSystem)
{
    cout << "Number of keyframes = " << ptrToORBSLAMSystem->GetNumberOfKeyframes() << endl;


}

unsigned long int GetNumberOfKeyFrames (ORB_SLAM2::System *ptrToORBSLAMSystem)
{
    return ptrToORBSLAMSystem->GetNumberOfKeyframes();
}

void shutDownORBSLAM (ORB_SLAM2::System *ptrToORBSLAMSystem, string filePathToSaveTrajectoryInfo, bool updateMapFile, string pathToSaveMapFile)
{
    ptrToORBSLAMSystem->Shutdown(pathToSaveMapFile);
    return;
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Helper Functions ////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ofstream openFeatureFile (string pathToFile)
{
    ofstream featurePointFile (pathToFile + "_KeypointFile.txt");
    return featurePointFile;
}


void validateAllMapPoints (ORB_SLAM2::System *ptrToSystem, int frameCounter)
{
    //Validation of our current map file with the default map file content

    //File stream to dump results
    ofstream outputFile;
    outputFile.open("MapPointDump_" + to_string(frameCounter) + ".txt");
    std::vector <ORB_SLAM2::MapPoint*> allMps = ptrToSystem->GetAllMapPoints();//Get all map points
    bool PRINT_EVERYTHING = true;//Reporting detail
    if (PRINT_EVERYTHING)
        outputFile << "Printing everything" << endl;

    unsigned int counter = 0;
    for (ORB_SLAM2::MapPoint *mp:allMps)//iterate throughout the whole map points in the map
    {
        outputFile << endl << "*************************" << endl << "***************" << endl << "Map point ID = " << mp->mnId << endl
        << "*************************" << endl << "***************" << endl;

        //Get observation lists from old and new map files
        std::map <ORB_SLAM2::KeyFrame*, size_t> obs = mp->GetObservations();
        std::map <ORB_SLAM2::KeyFrame*, MapPoint_Observation> mpObs = mp->mpObservation;


        if ( obs.size() != mpObs.size() )//if the number of observations in old and new map file are different
        {
            outputFile << "Size not equal" << endl;
            counter++;
            return;
        }
        else//when the number of observations are the same
        {
            //First print their key value pairs

            std::map <ORB_SLAM2::KeyFrame*, size_t>::iterator obsIterator = mp->GetObservationsUnlocked().begin();
            std::map <ORB_SLAM2::KeyFrame*, MapPoint_Observation>::iterator mpObsIterator = mp->mpObservation.begin();

            outputFile << "Number of elements = " << obs.size() << ", " << mpObs.size() << endl;
            outputFile << "Key value pairs" << endl;
            outputFile << "Obs KFs = ";
            for (; obsIterator !=  mp->GetObservationsUnlocked().end();)
            {
                ORB_SLAM2::KeyFrame* kf= obsIterator->first;
                if (kf->mnId == NULL && kf->mnId != 0)
                    outputFile << "NULL ID" << kf->mnId << ", ";
                else
                    outputFile << kf->mnId << ", ";

                ++obsIterator;
            }

            outputFile << endl << "Mps KFs = ";
            for (; mpObsIterator != mp->mpObservation.end(); mpObsIterator++)
            {
                if (mpObsIterator->first->mnId == NULL && mpObsIterator->first->mnId != 0)
                    outputFile << "NULL ID" << mpObsIterator->first->mnId << ", ";
                else
                 outputFile << mpObsIterator->first->mnId << ", ";
            }

            outputFile << endl;


            //Iterators
            obsIterator = mp->GetObservationsUnlocked().begin();
            mpObsIterator = mp->mpObservation.begin();

//            for (int keyframeCounter = 0; keyframeCounter < obs.size(); keyframeCounter++)
            int keyframeCounter = 0;

            // obsIterator !=  mp->GetObservations().end(), mpObsIterator != mp->mpObservation.end()
            while (keyframeCounter < mp->GetObservationsUnlocked().size())
//            for (; keyframeCounter < mp->GetObservations().size(); obsIterator++, mpObsIterator++, keyframeCounter++)


            {

                ORB_SLAM2::KeyFrame* currentKF = obsIterator->first;
                size_t index = obsIterator->second;

                outputFile << "*********************" << endl << "Iteration = " << keyframeCounter << endl;

                outputFile << "obsKF = " << currentKF->mnId << ", mpObsKF = " << mpObsIterator->first->mnId << endl;

                //Validating descriptors
                //Obs values
                outputFile << "The index is " << index << endl;
                cv::KeyPoint obsKP = currentKF->mvKeysUn[index];
                cv::Mat obsDescriptor = currentKF->mDescriptors.row(index);
                //mpObservation values
                ORB_SLAM2::KeyFrame* mpObservationKF = mpObsIterator->first;
                cv::Mat mpObservationDescriptor = mpObsIterator->second.descriptor;
                cv::KeyPoint mpObservationKeypoint = mpObsIterator->second.Keypoint;

                //Validate keyframes
                if (mpObservationKF->mnId != currentKF->mnId) {
                    outputFile << "Keyframes ID do not match" << endl << endl;
                    outputFile << "Printing all keyframe IDs here" << endl;


                    outputFile << "mpObservation = " << mpObservationKF->mnId << ", current KF = " << currentKF->mnId << endl;
                    outputFile << "Skipping this altogether" << endl;
//                    obsIterator++;
//
//
//                    outputFile << "Incremented again" << endl;
//                    //Validating descriptors
//                    //Obs values
//                    ORB_SLAM2::KeyFrame* currentKF = obsIterator->first;
//                    size_t index = obsIterator->second;
//                    outputFile << "The index is " << index << endl;
//                    cv::KeyPoint obsKP = currentKF->mvKeysUn[index];
//                    cv::Mat obsDescriptor = currentKF->mDescriptors.row(index);
//                    //mpObservation values
//                    ORB_SLAM2::KeyFrame* mpObservationKF = mpObsIterator->first;
//                    cv::Mat mpObservationDescriptor = mpObsIterator->second.descriptor;
//                    cv::KeyPoint mpObservationKeypoint = mpObsIterator->second.Keypoint;
//                    outputFile << "mpObservation = " << mpObservationKF->mnId << ", current KF = " << currentKF->mnId << endl;
//
//
//                    return;




                }
                else
                {
                    if (PRINT_EVERYTHING)
                        outputFile << "Keyframe ID = " << mpObservationKF->mnId << endl;
                    //Keyframe validated
                    //Now validate key points
                    if ( (obsKP.pt.x != mpObservationKeypoint.pt.x) && (obsKP.pt.y != mpObservationKeypoint.pt.y) )
                    {
                        counter++;
                        outputFile << "Keypoints do not match" << endl;
                    }
                    else
                    {
                        if (PRINT_EVERYTHING)
                        {
                            outputFile << "obsKP ( " << obsKP.pt.x << ", " << obsKP.pt.y << ")" << endl;
                            outputFile << "mpsKP ( " << mpObservationKeypoint.pt.x << ", " << mpObservationKeypoint.pt.y << ")" << endl;
                        }
                        //Key points validated
                        //Now validate descriptors
                        //Validate descriptors
                        if (GetDescriptorDistance (obsDescriptor, mpObservationDescriptor ) > 0 ) {
                            outputFile << "Descriptors do not match" << endl;
                            counter++;
                        }
                        else
                        {
                            if (PRINT_EVERYTHING)
                            {
                                outputFile << "Ob : " << obsDescriptor << endl;
                                outputFile << "mp : " << mpObservationDescriptor << endl;
                            }
                        }
                    }

                }



                ++mpObsIterator;

                keyframeCounter++;
                ++obsIterator;



            }

        }

        /*

        //First check if the number of elements in the three data structures for this map point are the same
        if ((mp->mpObservation.size() == mp->unKP.size()) && (mp->mpObservation.size() == mp->descriptorList.size()))
        {

            //Generate iterators for all three data structures
            std::map <ORB_SLAM2::KeyFrame*, MapPoint_Observation>::iterator mpObs = mp->mpObservation.begin();
            std::map <ORB_SLAM2::KeyFrame*, cv::KeyPoint>::iterator kpObs = mp->unKP.begin();
            std::map <ORB_SLAM2::KeyFrame*, cv::Mat>::iterator descObs = mp->descriptorList.begin();

            for (int counter = 0; counter < mp->unKP.size(); counter++)
            {
                outputFile <<  "KeyframeID = " << mpObs->first->mnId << endl;

                //Check if all data structures belong to the same keyframe, if not, cerr
                if ( (mpObs->first->mnId != descObs->first->mnId) || (mpObs->first->mnId != kpObs->first->mnId) || ((kpObs->first->mnId != kpObs->first->mnId)) )
                        outputFile << mpObs->first->mnId << ", " << descObs->first->mnId << ", " << kpObs->first->mnId << endl;

                //Descriptor validation
                if (GetDescriptorDistance(mpObs->second.descriptor, descObs->second) > 0) {
                    outputFile << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
                    outputFile << "mpObs = " << mpObs->second.descriptor << endl;
                    outputFile << "descObs = " << descObs->second << endl;
                }
                else
                {
                    if (PRINT_EVERYTHING) {
                        outputFile << "Same descriptors" << endl;
                        outputFile << mpObs->second.descriptor << endl;
                        outputFile << descObs->second << endl;
                    }
                }

                //Key point validation
                if ( (mpObs->second.Keypoint.pt.x != kpObs->second.pt.x) || (mpObs->second.Keypoint.pt.y != kpObs->second.pt.y) )
                {
                    outputFile << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
                    outputFile << "mpObs = " << mpObs->second.Keypoint.pt.x << ", " << mpObs->second.Keypoint.pt.y << endl;
                    outputFile << "kpObs = " << kpObs->second.pt.x << ", " << kpObs->second.pt.y << endl;
                }
                else
                {
                    if (PRINT_EVERYTHING) {
                        outputFile << "Key points are SAME" << endl;
                        outputFile << "mpObs = " << mpObs->second.Keypoint.pt.x << ", " << mpObs->second.Keypoint.pt.y
                                   << endl;
                        outputFile << "kpObs = " << kpObs->second.pt.x << ", " << kpObs->second.pt.y << endl;
                    }
                }
                mpObs++, kpObs++, descObs++;
            }
        }

        else//this is where the number of key points is more than other data structures
        {
            outputFile << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
            outputFile << "mpObservations = " << mp->mpObservation.size() << ", descriptor List = "
                 << mp->descriptorList.size() << ", unKP = " << mp->unKP.size() << endl;
            counter++;

            std::map <ORB_SLAM2::KeyFrame*, MapPoint_Observation>::iterator mpObs = mp->mpObservation.begin();
            std::map <ORB_SLAM2::KeyFrame*, cv::KeyPoint>::iterator kpObs = mp->unKP.begin();
            std::map <ORB_SLAM2::KeyFrame*, cv::Mat>::iterator descObs = mp->descriptorList.begin();


            cout << "mpObservation, descriptorList, unKP" << endl;
            bool printOnlyKP = false;
            for (int counter = 0; counter < mp->unKP.size(); counter++)
            {
                //if (printOnlyKP == false)
                {
                    outputFile << mpObs->first->mnId << ", " << descObs->first->mnId << ", " << kpObs->first->mnId << endl;

                    //We can compare descriptors here

                    //Descriptor validation
                    if (GetDescriptorDistance(mpObs->second.descriptor, descObs->second) > 0)
                        outputFile << "Descriptor are different";
                    else
                    {
//                        cout << mpObs->second.descriptor << endl;
//                        cout << descObs->second << endl;
//                        cout << "************************************" << endl;
                    }



                }
                //else
                  //  cout << "-, -, " << kpObs->first->mnId << endl;

//                if ((counter >= mp->mpObservation.size()-1) || (counter >= mp->descriptorList.size()-1)) {
//                    kpObs++;
//                    printOnlyKP = true;
//                }
//                else
                    mpObs++, kpObs++, descObs++;
            }


        }

        */
    }

    outputFile << "*************************" << endl << "***********************" << endl << "Anomolies = " << counter << endl;
}

int GetDescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

bool ConvertStringToBool (string inputStr)
{
    if (inputStr == "true")
        return true;
    else
        return false;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// Initialization Functions //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void InitPaths (cv::FileStorage fSettings, string &pathToSaveMap, string &pathToLoadMap,
                string &pathToSaveTrajectory,
                string &pathToSaveCameraCalib, string &pathToLoadCameraCalib,
                string kittiSequenceNumber, string loadKittiSequenceNumber,
                bool &dynamicObjectRemoval, const bool useBin)
{

    string suffix = "";
    if ( ConvertStringToBool(fSettings["Mode.RemoveDynamicObjects"]) )
    {
        suffix = "ON";
        dynamicObjectRemoval = true;
    }
    else
    {
        suffix = "OFF";
        dynamicObjectRemoval = false;
    }

    pathToSaveMap += kittiSequenceNumber;
    pathToLoadMap = loadKittiSequenceNumber;
    pathToSaveTrajectory += kittiSequenceNumber + ".txt";


    if (useBin == true)
    {
        pathToSaveMap += ".bin";
        pathToLoadMap += ".bin";
        pathToLoadCameraCalib += ".bin";
        pathToSaveCameraCalib += ".bin";
    }
    else
    {
        pathToLoadMap += ".txt";
        pathToSaveMap += ".txt";
        pathToLoadCameraCalib += ".txt";
        pathToSaveCameraCalib += ".txt";
    }


    cout << "Map save path: " << pathToSaveMap << endl;
    cout << "Map load path: " << pathToLoadMap << endl;
    cout << "Trajectory save path: " << pathToSaveTrajectory << endl;
    cout << "Filter = " << suffix << endl;

    return;

}

void InitMode (cv::FileStorage fSettings, const int opCode, int &startFrom, int &stopAt, bool &saveMap, bool &reuseMap, bool &useBin)
{
    string opCodeStr;
    switch (opCode)
    {
        case 0:
            QS_State = SAVE_MAP;
            cout << "CarMap_State = SAVE_MAP" << endl;
            opCodeStr = "SaveMap.";
            break;

        case 1:
            QS_State = UPDATE_MAP;
            cout << "CarMap_State = UPDATE_MAP" << endl;
            opCodeStr = "UpdateMap.";
            break;

        case 2:
            QS_State = LOAD_MAP;
            cout << "CarMap_State = LOAD_MAP" << endl;
            opCodeStr = "LoadMap.";
            break;

        default:
            cout << "Shouldn't be here" << endl;
            break;

    }

    startFrom = fSettings[opCodeStr + "StartFrom"];
    stopAt = fSettings[opCodeStr + "StopAt"];
    saveMap = ConvertStringToBool ( fSettings[opCodeStr + "SaveMap"] );
    reuseMap = ConvertStringToBool ( fSettings [opCodeStr + "ReuseMap"] );
    useBin = ConvertStringToBool ( fSettings [opCodeStr + "UseBin" ] );

    return;
}


std::string GetMatType(const cv::Mat& mat)
{
    const int mtype = mat.type();

    switch (mtype)
    {
        case CV_8UC1:  return "CV_8UC1";
        case CV_8UC2:  return "CV_8UC2";
        case CV_8UC3:  return "CV_8UC3";
        case CV_8UC4:  return "CV_8UC4";

        case CV_8SC1:  return "CV_8SC1";
        case CV_8SC2:  return "CV_8SC2";
        case CV_8SC3:  return "CV_8SC3";
        case CV_8SC4:  return "CV_8SC4";

        case CV_16UC1: return "CV_16UC1";
        case CV_16UC2: return "CV_16UC2";
        case CV_16UC3: return "CV_16UC3";
        case CV_16UC4: return "CV_16UC4";

        case CV_16SC1: return "CV_16SC1";
        case CV_16SC2: return "CV_16SC2";
        case CV_16SC3: return "CV_16SC3";
        case CV_16SC4: return "CV_16SC4";

        case CV_32SC1: return "CV_32SC1";
        case CV_32SC2: return "CV_32SC2";
        case CV_32SC3: return "CV_32SC3";
        case CV_32SC4: return "CV_32SC4";

        case CV_32FC1: return "CV_32FC1";
        case CV_32FC2: return "CV_32FC2";
        case CV_32FC3: return "CV_32FC3";
        case CV_32FC4: return "CV_32FC4";

        case CV_64FC1: return "CV_64FC1";
        case CV_64FC2: return "CV_64FC2";
        case CV_64FC3: return "CV_64FC3";
        case CV_64FC4: return "CV_64FC4";

        default:
            return "Invalid type of matrix!";
    }
}


