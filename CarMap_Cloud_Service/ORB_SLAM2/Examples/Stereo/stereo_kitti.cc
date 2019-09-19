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
        pathToSaveCameraCalib, pathToLoadCameraCalib;

int startFrom, stopAt, opCode, loadDepth;
bool saveMap, reuseMap, useBin, runLocalizationMode, saveTrajectory, isStereo, ORBVisualization, evaluateSegmentation, majorityVoting, robustFeatureSearch;
///////////////////////////////////////////////////
// Map upload
const string serverAddr = "204.57.3.131";
const int portNumber = 2000;
const bool isUploadMap = false;
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

int main(int argc, char **argv) {

    // Parse the configuration file
    ReadAndParseConfigFile (pathToConfigFile);


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

    ORB_SLAM2::System *SLAM = initializeORBSlam(pathToVocabFile, pathToSettingsFile,
                                                ORBVisualization, saveMap,
                                                reuseMap, pathToLoadMapFile,
                                                runLocalizationMode, saveTrajectory,
                                                pathToSaveTrajectory, isStereo,
                                                serverAddr, portNumber,
                                                isUploadMap, viewWindowName,
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
        if (loadDepth) {
            imDepth = cv::imread(vstrDepthImages[ni], CV_LOAD_IMAGE_UNCHANGED);
            if (imDepth.data == NULL)
                cout << "Data is null" << endl;
        }

        double tframe = vTimestamps[ni];
        if (imLeft.empty()) {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
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


    }


    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    cout << "Total time = " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000000.0f << endl;
    cout << "Total tracking time = " << ( total_tracking_time / 1000000.0f)  << endl;
    cout << "Per frame processing time = " << ( total_tracking_time / 1000000.0f) / (stopAt - startFrom) << endl;

/*




    // Tracking time statistics
    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for (int ni = 0; ni < nImages; ni++) {
        totaltime += vTimesTrack[ni];
    }

//
//    unsigned int labelCounters [ SEMANTIC_SEGMENTOR_NUMBER_OF_OBJECTS ] = {0};
//    SLAM->GetMap()->PrintMapPointLabelStatistics(labelCounters);
 */
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

    const string imageExt = ".png";
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "clean_times.txt";
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
    pathToProject = (string) fSettings["Path.PathToProject"];
    pathToMapFileDirectory = (string) fSettings["Path.PathToMapFileDirectory"];
    dataDirectory = (string) fSettings["Path.PathToDataDirectory"];
    pathToSettingsFile = pathToProject + (string) fSettings["Path.PathToCameraCalibrationFile"];
    pathToVocabFile = pathToProject + (string) fSettings["Path.PathToVocabularyFile"];

    seqNo = (string) fSettings["Path.SequenceNumber"];
    loadMapSeqNo = (string) fSettings["Path.LoadMapSequenceNumber"];
    pathToSeq = (string) fSettings["Path.PathToInputImages"] + seqNo + "/";
    kittiSequenceNumber = "Seq" + seqNo;

    pathToSaveMapFile = pathToMapFileDirectory;
    pathToLoadMapFile = pathToMapFileDirectory;
    pathToSaveTrajectory = dataDirectory + (string) fSettings["Path.PathToSaveTrajectory"];

    // Constants
    runLocalizationMode = ConvertStringToBool (fSettings["Constant.RunLocalization"]);
    saveTrajectory = ConvertStringToBool (fSettings["Constant.SaveTrajectory"]);
    isStereo = ConvertStringToBool (fSettings["Constant.IsStereo"]);
    ORBVisualization = ConvertStringToBool (fSettings["Constant.OrbVisualization"]);
    loadDepth = ConvertStringToBool( fSettings["Mode.LoadDepth"] );
    evaluateSegmentation = ConvertStringToBool ( fSettings["Mode.EvaluateSegmentation"] );
    majorityVoting = ConvertStringToBool ( fSettings ["Mode.MajorityVoting"] );
    robustFeatureSearch = ConvertStringToBool ( fSettings ["Mode.RobustFeatureSearch"] );

    opCode = fSettings["OperationMode.OpCode"];
    InitSystem(fSettings, opCode);
}