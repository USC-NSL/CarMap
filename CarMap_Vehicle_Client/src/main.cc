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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>
#include <GlobalDefintions.h>
#include<opencv2/core/core.hpp>

#include<System.h>
#include <include/ZEDInitialization.h>

using namespace std;

string pathToProject = "../";
string pathToConfigFile = pathToProject + "Config_File.yaml";
string pathToMapFileDirectory, dataDirectory,
        pathToSettingsFile, pathToVocabFile,
        seqNo, loadMapSeqNo, pathToSeq,
        kittiSequenceNumber, pathToSaveMapFile,
        pathToLoadMapFile, pathToSaveTrajectory,
        pathToSaveCameraCalib, pathToLoadCameraCalib, path_to_e2e_latency_log;

int startFrom, stopAt, opCode, loadDepth, upload_frame_interval;
bool saveMap, reuseMap, useBin, runLocalizationMode, saveTrajectory, isStereo, ORBVisualization, evaluateSegmentation, majorityVoting, robustFeatureSearch, log_e2e_latency, upload_only_diff;
///////////////////////////////////////////////////
// Map upload
string cloud_service_ip_address;
int cloud_service_port_number;
bool is_upload_map_to_cloud = false;
bool dynamicObjectRemoval;

void LoadImages(const string &strPathToSequence,
                vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight,
                vector <string> &vstrLabels,
                vector <string> &vstrOverlay,
                vector <string> &vstrDepth,
                vector <string> &vstrDP,
                vector <string> &vstrMN,
                vector <string> &vstrFDP,
                vector<double> &vTimestamps);

void InitSystem(cv::FileStorage fSettings, int opCode);
void ReadAndParseConfigFile ( string pathToConfigFile );
void DisplayOperationMode ();

int main(int argc, char **argv) {

    // Parse the configuration file
    ReadAndParseConfigFile (pathToConfigFile);
    DisplayOperationMode ();


    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    vector<string> vstrLabels;
    vector<string> vstrOverlayImages;
    vector<string> vstrDepthImages;
    vector<string> vstrDeepLabLabels;
    vector<string> vstrMnv2Labels;
    vector<string> vstrTunedDeepLabLabels;

    cout << "Reading images" << endl;
    LoadImages(pathToSeq, vstrImageLeft, vstrImageRight,  vstrLabels, vstrOverlayImages, vstrDepthImages,
               vstrDeepLabLabels, vstrMnv2Labels, vstrTunedDeepLabLabels,  vTimestamps);



    const int nImages = vstrImageLeft.size();
    cout << "Number of images = " << nImages << endl;
    cout << "Number of labels = " << vstrLabels.size() << endl;
    cout << "Number of overlays = " << vstrOverlayImages.size() << endl;

    string viewWindowName = "SLAM One";
    bool isReconstructMap = true;
    bool isStitchMap = false;

    std::ofstream e2e_logger;

    if (log_e2e_latency == true)
    {
        cout << path_to_e2e_latency_log << endl;
        e2e_logger.open(path_to_e2e_latency_log, std::ofstream::out | std::ofstream::app);
        if (e2e_logger.is_open() == false)
        {
            cerr << "Could not open e2e log file at: " << path_to_e2e_latency_log << endl;
            return -1;
        }
        else
            cout << "Opened e2e latency file at: " << path_to_e2e_latency_log << endl;

        e2e_logger << "Vehicle_Counter,Vehicle_to_Cloud_Time,Vehicle_to_Cloud_Map_Size,Cloud_to_Vehicle_Map_Size,Erroreous_E2E_Latency" << endl;

    }

    ORB_SLAM2::System *SLAM = initializeORBSlam(pathToVocabFile, pathToSettingsFile,
                                                ORBVisualization, saveMap,
                                                reuseMap, pathToLoadMapFile,
                                                runLocalizationMode, saveTrajectory,
                                                pathToSaveTrajectory, isStereo,
                                                cloud_service_ip_address, cloud_service_port_number,
                                                is_upload_map_to_cloud, viewWindowName,
                                                isReconstructMap, isStitchMap,
                                                dynamicObjectRemoval, majorityVoting,
                                                robustFeatureSearch);


    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imLeft, imRight;
    cv::Mat imLabel;
    cv::Mat imOverlay;
    cv::Mat imDepth;
    cv::Mat imDP, imMN, imFDP;

    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    float total_tracking_time = 0;
    for (int ni = startFrom; ni < stopAt; ni++) {
        cout << ni << " / " << nImages << endl;
        // Read images
        imLeft = cv::imread(vstrImageLeft[ni], CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni], CV_LOAD_IMAGE_UNCHANGED);
        imLabel = cv::imread (vstrLabels[ni], CV_LOAD_IMAGE_UNCHANGED);
        imOverlay = cv::imread (vstrOverlayImages[ni], CV_LOAD_IMAGE_UNCHANGED);
        imDP = cv::imread (vstrDeepLabLabels[ni], CV_LOAD_IMAGE_UNCHANGED);
        imMN = cv::imread (vstrMnv2Labels[ni], CV_LOAD_IMAGE_UNCHANGED);
        imFDP = cv::imread (vstrTunedDeepLabLabels[ni], CV_LOAD_IMAGE_UNCHANGED);

        if (evaluateSegmentation)
        if (imDP.data == NULL || imMN.data ==NULL || imFDP.data == NULL )
        {
            cout << "Data is NULL!";
            cin.get();
        }

        if (loadDepth)
        {
            imDepth = cv::imread(vstrDepthImages[ni], CV_LOAD_IMAGE_UNCHANGED);
            if (imDepth.data == NULL)
                cout << "Data is null" << endl;
        }

        double tframe = vTimestamps[ni];
        if (imLeft.empty() || imRight.empty() || imLabel.empty()) {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl
                 << string (vstrImageRight[ni]) << endl
                 << string (vstrLabels[ni]) << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
        if (loadDepth)
            SLAM->TrackStereo(imLeft, imRight, imLabel, imOverlay, imDepth, tframe);
        else if (evaluateSegmentation)
        {
            SLAM->TrackStereo (imLeft, imRight, imLabel, imOverlay, imDP, imMN, imFDP, tframe );
        }
        else
            SLAM->TrackStereo(imLeft, imRight, imLabel, imOverlay, tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        total_tracking_time += std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();

        if (ni % upload_frame_interval == 0 && is_upload_map_to_cloud == true)
        {
            cout << "CarMap checkpoint: Going to send a map update and reset mapping all over again" << endl;
            SLAM->SendMapAndReset(&e2e_logger, !upload_only_diff);
            cout << "Starting CarMap from scratch" << endl;
        }
    }

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    cout << "Total time = " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000000.0f << endl;
    cout << "Total tracking time = " << ( total_tracking_time / 1000000.0f)  << endl;
    cout << "Per frame processing time = " << ( total_tracking_time / 1000000.0f) / (stopAt - startFrom) << endl;

    SLAM->GetMap()->PrintResultsFromDiffOperation();
    cout << "Shutting down ..." << endl;
    sleep (2);
    if (saveTrajectory == true)
        SLAM->SaveTrajectoryKITTI(pathToSaveTrajectory);
    shutDownORBSLAM(SLAM, "", true, pathToSaveMapFile);
    return 0;
}


void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<string> &vstrLabel,
                vector<string> &vstrOverlay, vector<string> &vstrDepth,
                vector<string> &vstrDPLabels, vector<string>&vstrMNLabels,
                vector<string>&vstrFDPLabels, vector<double> &vTimestamps)
                {

    cout << "Locating images ...." << endl;
    cout << "Path to seq = " << strPathToSequence << endl;
    const string imageExt = ".png";
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "times.txt";
    int counter = 0;
    cout << "Path to time file: " << strPathTimeFile << endl;

    fTimes.open(strPathTimeFile.c_str());
    if (fTimes.is_open() == false)
    {
        cerr << "Invalid path to time file: " << strPathTimeFile.c_str () << endl;
        cerr << "Could not open time file" << endl;
        return;
    }
    while (!fTimes.eof()) {
        string s;
        getline(fTimes, s);
        if (!s.empty()) {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }


    string strPrefixLeft = strPathToSequence + "image_2/";
    string strPrefixRight = strPathToSequence + "image_3/";
    string strPrefixLabel = strPathToSequence + "segmentation/";
    string strPrefixOverlay = strPathToSequence + "image_2/";
    string strPrefixDepth = strPathToSequence + "depth/";
    string strDPLabel = strPathToSequence + "deeplabv3/segmented_image_2/";
    string strMNLabel = strPathToSequence + "mnv2/segmented_image_2/";
    string strFDPLabel = strPathToSequence + "tuned_deeplabv3/segmented_image_2/";


    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);
    vstrLabel.resize(nTimes);
    vstrOverlay.resize(nTimes);
    vstrDepth.resize(nTimes);
    vstrDPLabels.resize(nTimes);
    vstrMNLabels.resize(nTimes);
    vstrFDPLabels.resize(nTimes);

    for (int i = 0; i < nTimes; i++) {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;

        vstrImageLeft[i] = strPrefixLeft + ss.str() + imageExt;
        vstrImageRight[i] = strPrefixRight + ss.str() + imageExt;
        vstrLabel[i] = strPrefixLabel + "segmented_" + ss.str() + imageExt;

        if (loadDepth) {
            vstrDepth[i] = strPrefixDepth + ss.str() + imageExt;
        }
        if (evaluateSegmentation) {
            vstrDPLabels[i] = strDPLabel + "segmented_" + ss.str() + imageExt;
            vstrMNLabels[i] = strMNLabel + "segmented_" + ss.str() + imageExt;
            vstrFDPLabels[i] = strFDPLabel + "segmented_" + ss.str() + imageExt;
        }
        vstrOverlay[i] = strPrefixOverlay  + ss.str() + imageExt;

    }
    cout << "Located images" << endl;
}
void InitSystem(cv::FileStorage fSettings, int opCode) {

    InitMode (fSettings, opCode, startFrom, stopAt, saveMap, reuseMap, useBin);
    InitPaths (fSettings,
               pathToSaveMapFile, pathToLoadMapFile,
               pathToSaveTrajectory,
               pathToSaveCameraCalib, pathToLoadCameraCalib,
               kittiSequenceNumber, loadMapSeqNo,
               dynamicObjectRemoval, useBin);


}
void ReadAndParseConfigFile ( string pathToConfigFile )
{
    cv::FileStorage fSettings(pathToConfigFile, cv::FileStorage::READ);

    if(!fSettings.isOpened())
    {
        cerr << "Failed to open file " << pathToConfigFile << endl;
        return;
    }

    // Path variables
    pathToProject = "../";
    dataDirectory = (string) fSettings["Path.PathToResultsDirectory"];
    pathToSettingsFile = pathToProject + (string) fSettings["Path.PathToCameraCalibrationFile"];
    pathToMapFileDirectory = pathToProject + dataDirectory;
    cout << pathToMapFileDirectory << endl;
    pathToVocabFile = pathToProject + (string) fSettings["Path.PathToVocabularyFile"];

    seqNo = (string) fSettings["Path.Dataset"];
    loadMapSeqNo = pathToProject + dataDirectory + "/" +  (string) fSettings["Path.LoadMap"];
    pathToSeq = pathToProject + seqNo + "/";
    kittiSequenceNumber = seqNo;
    upload_frame_interval = fSettings ["Constant.UploadFrameInterval"];

    pathToSaveMapFile = pathToMapFileDirectory + "/" ;
    pathToLoadMapFile = loadMapSeqNo;
    pathToSaveTrajectory = pathToProject + dataDirectory + "/";
    path_to_e2e_latency_log = (string) fSettings ["Path.E2ELatencyLog"];

    // Constants
    runLocalizationMode = ConvertStringToBool (fSettings["Constant.RunLocalization"]);
    saveTrajectory = ConvertStringToBool (fSettings["Constant.SaveTrajectory"]);
    isStereo = ConvertStringToBool (fSettings["Constant.IsStereo"]);
    ORBVisualization = ConvertStringToBool (fSettings["Constant.OrbVisualization"]);
    cloud_service_ip_address = (string) fSettings ["Constant.CloudIPAddress"] ;
    cloud_service_ip_address = cloud_service_ip_address.substr ( cloud_service_ip_address.find ('.') + 1, cloud_service_ip_address.size() );
    cout << "IP address is: " << cloud_service_ip_address << endl;
    cloud_service_port_number = fSettings ["Constant.CloudPort"];

    loadDepth = ConvertStringToBool( fSettings["Mode.LoadDepth"] );
    evaluateSegmentation = ConvertStringToBool ( fSettings["Mode.EvaluateSegmentation"] );
    majorityVoting = ConvertStringToBool ( fSettings ["Mode.MajorityVoting"] );
    robustFeatureSearch = ConvertStringToBool ( fSettings ["Mode.RobustFeatureSearch"] );
    is_upload_map_to_cloud = ConvertStringToBool ( fSettings ["Mode.UploadToCloud"] );
    log_e2e_latency = ConvertStringToBool (fSettings ["Mode.LogE2ELatency"]);
    upload_only_diff = ConvertStringToBool (fSettings ["Mode.UploadOnlyDiff"]);

    opCode = fSettings["OperationMode.OpCode"];
    InitSystem(fSettings, opCode);
}
void DisplayOperationMode (void)
{
    cout << "************************" << endl;
    cout << "************************" << endl;
    cout << "************************" << endl;
    cout << "CarMap Operation Mode: " << endl;
    cout << "Input dataset: " << pathToSeq << endl;
    cout << "Sequence range: " << startFrom << " - " << stopAt << endl;
    cout << "Calibration file: " << pathToSettingsFile << endl;
    cout << "Vocab file: " << pathToVocabFile << endl;
    cout << "Save trajectory: " << (saveTrajectory ? "True" : "False") << endl;
    cout << "Visualization: " << (ORBVisualization ? "True" : "False") << endl;
    cout << "Remove dynamic objects: " << (dynamicObjectRemoval ? "True" : "False") << endl;
    cout << "Evaluate semantic segmentation: " << ( evaluateSegmentation ? "True" : "False" ) << endl;
    cout << "Majority voting in segmentation: " << ( majorityVoting ? "True" : "False" ) << endl;
    cout << "Robust feature search: " << (robustFeatureSearch ? "True" : "False") << endl;
    cout << "Upload to cloud service: " << (is_upload_map_to_cloud ? "True" : "False") << endl;
    cout << "Log E2E latency: " << (log_e2e_latency ? "True" : "False") << endl;
    cout << "CarMap vehicle operation: " << ( upload_only_diff ? "Map update" : "Map stitch" ) << endl;
    cout << "************************" << endl;
    cout << "************************" << endl;
    cout << "************************" << endl;
}
