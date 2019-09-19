//
// Created by  on 9/27/18.
//

// Header files to include
#include <iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>
#include<opencv2/core/core.hpp>
#include<System.h>
#include <include/ZEDInitialization.h>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/asio.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/vector.hpp>



// Path variables
string pathToProject = "../"; //path to the project
const string pathToConfigFile = pathToProject + "Config_File.yaml";

string dataDirectory, pathToMapFileDirectory,
        pathToSettingsFile, pathToVocabFile,
        pathToMapOne, pathToMapTwo,
        pathToSaveMap, pathToSaveTrajectory, path_to_log_file;

// Map strings
const string mapOneStr = "Base map";
const string mapTwoStr = "Second map";

string pathToSaveCameraCalib;
string pathToLoadCameraCalib;

// Control bools
const bool runLocalizationMode = false;
const bool saveTrajectory = true;
const bool isStereo = true;
//const bool ORBVisualization = true;
const bool saveMap = true;
const bool reuseMap = true;
const bool saveMapOne = false;
const bool saveBaseMap = true;
const bool reconstructBaseMap = true;
const bool reconstructLostMap = false;
const bool isDebugMode = true;
bool oneShotStitching, visualizeMapOne, visualizeMapTwo, ORBVisualization;
bool receive_vehicle_map_over_network, integrate_diff;
string cloud_service_ip_address;
int cloud_service_port_number;

///////////////////////////////////////////////////
// Map upload
const string serverAddr = "127.0.0.1";
const string client_ip_address = "127.0.0.1";
const int portNumber = 2000;
const bool isUploadMap = false;
const int FirstKeyFrameIndex = 1;
const float BYTES_TO_MB = 0.000001;




float get3Ddistance (float x, float y, float z)
{
    return sqrt( pow(x,2) + pow (y,2) + pow (z,2));
}

cv::Mat getTransformation (cv::Mat initialPos, cv::Mat finalPos)
{
    cv::Mat error(4, 4, CV_32F);
    error.at<float>(0,3) = finalPos.at<float>(0,3) - initialPos.at<float>(0,3);
    error.at<float>(1,3) = finalPos.at<float>(1,3) - initialPos.at<float>(1,3);
    error.at<float>(2,3) = finalPos.at<float>(2,3) - initialPos.at<float>(2,3);

    return error;
}

ORB_SLAM2::System *InitBaseMap (string pathToBaseMap, bool visualizeMap, string viewerName, bool reconstructMap)
{
	cout << "Vocab file: " << pathToVocabFile << endl;
	cout << "Path to settings file: " << pathToSettingsFile << endl;
    ORB_SLAM2::System *baseMap = initializeORBSlam(pathToVocabFile, pathToSettingsFile,
                                                   visualizeMap, saveMap, reuseMap, pathToBaseMap,
                                                   runLocalizationMode, saveTrajectory, pathToSaveTrajectory,
                                                   isStereo, serverAddr, portNumber, isUploadMap,
                                                   viewerName, reconstructMap, true, false);

    return baseMap;
}

ORB_SLAM2::System *InitSecondMap (string pathToSecondMap, bool visualizeMap, string viewerName, bool reconstructMap)
{
    bool saveMap = false;
    ORB_SLAM2::System *relocalizeMap = initializeORBSlam(pathToVocabFile, pathToSettingsFile,
                                                         visualizeMap, saveMap, reuseMap, pathToSecondMap,
                                                         runLocalizationMode, saveTrajectory, pathToSaveTrajectory,
                                                         isStereo, serverAddr, portNumber, isUploadMap,
                                                         viewerName, reconstructMap, false, false);

    return relocalizeMap;
}

ORB_SLAM2::System* InitReceivedMap (ORB_SLAM2::Map *map, bool reconstructMap)
{
    bool save_map = true;



}

void PrintNumberOfMapElements (ORB_SLAM2::System *map, const string mapStr)
{
    if (isDebugMode)
        cout << mapStr << " : Keyframes = " << map->GetAllKeyFrames().size() << ", map-points = " << map->GetAllMapPoints().size() << endl;
}

ORB_SLAM2::KeyFrame *GetKeyFrame (const int index, vector <ORB_SLAM2::KeyFrame*> keyframes)
{
    for (int counter = FirstKeyFrameIndex; counter < keyframes.size(); counter++)
    {
            if (keyframes.at(counter)->GetmnId() == index)
            {
                if (isDebugMode)
                    cout << "Keyframe to add = " << keyframes.at(counter)->GetmnId() << endl;
                return keyframes.at(counter);
            }
    }

    return static_cast<ORB_SLAM2::KeyFrame*> (NULL);

}

void InitStitchingModule (bool stitchingMode)
{
    if (isDebugMode)
        cout << "Initializing map stitching module" << endl;

    if ( stitchingMode )
        pathToSaveMap += "OneShot.bin";
    else
        pathToSaveMap += "Progressive.bin";

}

size_t ReceiveMap (boost::asio::ip::tcp::socket* carmap_socket, boost::asio::streambuf *buf)
{
    // receive data from carmap_socket and store it in buf
    size_t header;
    // read header
    boost::asio::read(*carmap_socket, boost::asio::buffer(&header, sizeof(header)));
    // read body
    const size_t received_map_size = boost::asio::read(*carmap_socket, buf->prepare(header));
    buf->commit(header);
    return received_map_size;
}

void DeserializeMap (boost::asio::streambuf* buf, ORB_SLAM2::Map* map_to_stitch)
{
    // deserialize
    std::istream is(buf);
    boost::archive::binary_iarchive ar(is);
    ar & map_to_stitch;
}

float StitchOperation (ORB_SLAM2::System* base_map, ORB_SLAM2::Map* map_to_stitch)
{
    // Start stitching away!
    // Get all the keyframes from the map to be augmented
    vector<ORB_SLAM2::KeyFrame *> lostKFs = map_to_stitch->GetAllKeyFrames();
    cout << "Number of keyframes received = " << lostKFs.size() << endl;
    cout << "Number of map-points received = " << map_to_stitch->GetAllMapPoints().size() << endl;
    std::chrono::steady_clock::time_point stitching_begin = std::chrono::steady_clock::now();
    // Main program loop
    // Iterate through all the keyframes from the map to be augmented
    const int NumberOfKeyFrames = lostKFs.size();
    for (int keyframeCounter = FirstKeyFrameIndex; keyframeCounter < NumberOfKeyFrames; keyframeCounter++)
    {
        //cout << "Actual number of keyframes / NumberOfKeyFrames " << endl;
        cout << keyframeCounter << " / " << NumberOfKeyFrames << endl;
        ORB_SLAM2::KeyFrame *kf = GetKeyFrame(keyframeCounter,
                                              lostKFs);// Get the next keyframe to add to the base map
        if (kf == NULL)// continue if no keyframe exists
        {
            if (isDebugMode)
                cout << "No keyframe with id = " << keyframeCounter << endl;
            continue;
        }
        // Stitch operation
        cv::Mat estimatedPose = base_map->TrackKeyFrame(kf, oneShotStitching); // Predict the pose and add the current keyframe to the base map
        if (estimatedPose.empty()) // continue if pose is un-defined
        {
            if (isDebugMode)
                cout << "Could not estimate pose for keyframe = " << kf->GetmnId() << endl;
            continue;
        }
    }
    std::chrono::steady_clock::time_point stitching_end = std::chrono::steady_clock::now();
    float stitching_time = (std::chrono::duration_cast<std::chrono::microseconds>(stitching_end - stitching_begin).count()) / 1000000.0f;
    std::cout << "Total stitching time = " << stitching_time << " seconds" << endl;
    return stitching_time;
}

void SerializeMap (ORB_SLAM2::Map* map_to_serialize, boost::asio::streambuf* write_buf)
{
    std::ostream os(write_buf);
    boost::archive::binary_oarchive write_ar(os);
    try
    {
        write_ar & map_to_serialize;
    }
    catch (boost::system::system_error e)
    {
        std::cerr << boost::diagnostic_information(e);
    }
}

float SendMap (boost::asio::ip::tcp::socket* carmap_socket, boost::asio::streambuf* write_buf)
{
    const size_t write_header = write_buf->size();
    // send header and buffer using scatter
    std::vector<boost::asio::const_buffer> write_buffers;
    write_buffers.push_back(boost::asio::buffer(&write_header, sizeof(write_header)));
    write_buffers.push_back(write_buf->data());
    size_t data_sent_in_bytes = 0;
    std::chrono::steady_clock::time_point tx_begin = std::chrono::steady_clock::now();
    try
    {
        const size_t rc = boost::asio::write(*carmap_socket, write_buffers);
        data_sent_in_bytes = rc;
    }
    catch (boost::system::system_error e)
    {
        std::cerr << boost::diagnostic_information(e);
    }
    std::chrono::steady_clock::time_point tx_end = std::chrono::steady_clock::now();
    float map_sent_size = data_sent_in_bytes * BYTES_TO_MB;
    return map_sent_size;
}

void ReadAndParseConfigFile (cv::FileStorage fSettings)
{
    // Parse configuration file
    pathToProject = "../";
    dataDirectory = pathToProject;
    pathToMapFileDirectory = dataDirectory + (string) fSettings["ServerStitch.PathToMapDirectory"];
    pathToSettingsFile = pathToProject + (string) fSettings["ServerStitch.PathToCameraCalibrationFile"];
    pathToVocabFile = pathToProject + (string) fSettings["ServerStitch.PathToVocabularyFile"];
    pathToMapOne = pathToMapFileDirectory + (string) fSettings["ServerStitch.PathToBaseMap"];
    pathToMapTwo = pathToMapFileDirectory + (string) fSettings["ServerStitch.PathToSecondMap"];
    pathToSaveMap = pathToMapFileDirectory + (string) fSettings["ServerStitch.PathToFinalMap"];
    path_to_log_file = (string) fSettings ["ServerStitch.PathToLogFile"];
    oneShotStitching = ConvertStringToBool ( (string) fSettings["ServerStitch.OneShotStitching"]);
    visualizeMapOne = ConvertStringToBool( (string) fSettings["ServerStitch.VisualizeBaseMap"] );
    visualizeMapTwo = ConvertStringToBool( (string) fSettings["ServerStitch.VisualizeSecondMap"] );
    ORBVisualization = ConvertStringToBool( (string) fSettings["ServerStitch.VisualizeStereoFrames"] );
    integrate_diff = ConvertStringToBool( (string) fSettings["ServerStitch.IntegrateDiff"] );
    receive_vehicle_map_over_network = ConvertStringToBool ( (string) fSettings["ServerStitch.ReceiveMaps"] );
    cloud_service_ip_address = (string) fSettings ["ServerStitch.CloudServiceIPAddress"];
    cloud_service_ip_address = cloud_service_ip_address.substr ( cloud_service_ip_address.find('.') + 1, cloud_service_ip_address.size() );
    cloud_service_port_number = fSettings ["ServerStitch.CloudServicePort"];

}

void IntegrateDiffInMap (ORB_SLAM2::System *base_map_system_ptr, ORB_SLAM2::Map *diff_map_ptr) {
    const bool is_debug_mode = false;
    ///////////////////////////
    // Print of the diff map

    if (is_debug_mode) {
        cout << "Integrating the diff in the map ...." << endl;
        cout << "Diff map statistics ...." << endl;
        cout << "Keyframes: " << diff_map_ptr->GetAllKeyFrames().size() << endl;
        cout << "Map-points: " << diff_map_ptr->GetAllMapPoints().size() << endl;
    }

    ////////////////////////////////
    ORB_SLAM2::Map *base_map_ptr = base_map_system_ptr->GetMap(); // pointer to the base map
    vector<ORB_SLAM2::KeyFrame *> base_map_kfs = base_map_ptr->GetAllKeyFrames();
    vector<ORB_SLAM2::KeyFrame *> diff_map_kfs = diff_map_ptr->GetAllKeyFrames();


    //////////////////////////////////////////
    // Order keyframes in both maps for an easier lookup
    if (is_debug_mode)
        cout << "Ordering keyframes in the base map for easy lookup" << endl;
    // First order keyframes in the base map
    std::map<long unsigned int, ORB_SLAM2::KeyFrame *> ordered_keyframes_in_base_map;
    for (ORB_SLAM2::KeyFrame *kf : base_map_kfs) {
        if (kf != NULL)
            if (kf->isBad() == false)
                ordered_keyframes_in_base_map.insert(
                        std::pair<unsigned long int, ORB_SLAM2::KeyFrame *>(kf->GetmnId(), kf));
    }
    // Order keyframes in the diff map
    std::map<long unsigned int, ORB_SLAM2::KeyFrame *> ordered_keyframes_in_diff_map;
    for (ORB_SLAM2::KeyFrame *kf : diff_map_kfs) {
        if (kf != NULL)
            if (kf->isBad() == false)
                ordered_keyframes_in_diff_map.insert(
                        std::pair<unsigned long int, ORB_SLAM2::KeyFrame *>(kf->GetmnId(), kf));
    }

    // Link map-points with their corresponding keyframes in the diff map
    if (is_debug_mode)
        cout << "Adding new map-points to the current base map ...." << endl;

    vector<ORB_SLAM2::MapPoint *> all_new_mps = diff_map_ptr->GetAllMapPoints();
    int mp_counter = 0;
    for (ORB_SLAM2::MapPoint *new_mp : all_new_mps) {
        // Get the reference keyframe for this map point from its mnId
        unsigned long int ref_kf_lookup_id = new_mp->GetmnId();
        // The reference keyframe should not exist in the base map
        if (ordered_keyframes_in_base_map.find(ref_kf_lookup_id) == ordered_keyframes_in_base_map.end()) {
            // Since its a new keyframe, it should exist in the diff map
            if (ordered_keyframes_in_diff_map.find(ref_kf_lookup_id) != ordered_keyframes_in_diff_map.end()) {
                // Now, set the keyframe as the reference keyframe for the current map-point
                new_mp->ReplaceReferenceKeyFrame(ordered_keyframes_in_diff_map.find(ref_kf_lookup_id)->second);
                // Also, set a pointer from the keyframe to the map-point as well
                ordered_keyframes_in_diff_map.find(ref_kf_lookup_id)->second->AddMapPoint(new_mp);
            } else {
                // the keyframe does not exist in the base map nor the diff map
                cerr << "KF [" << mp_counter << "] does not exist in the base map or diff map ....." << endl;
            }
        } else {
            cerr << "KF [" << mp_counter << "] already exists in the base map ....." << endl;
        }
    }

    if (is_debug_mode)
    {
        cout << "Base map statistics before integration ...." << endl;
        cout << "Keyframes: " << base_map_system_ptr->GetAllKeyFrames().size() << ", map-points: "
             << base_map_system_ptr->GetAllMapPoints().size() << endl;
    }
    // Print the number of map-points associated with each keyframe now
    for ( ORB_SLAM2::KeyFrame* kf : diff_map_kfs )
    {
        cout << kf->GetmnId() << " has " << kf->GetMapPoints().size() << endl;
        ORB_SLAM2::KeyFrame* new_kf_in_base_map = new ORB_SLAM2::KeyFrame ( kf, base_map_ptr, base_map_system_ptr->GetKeyFrameDatabase() );


        // Add the keyframe to the base map
        base_map_ptr->AddKeyFrame(new_kf_in_base_map);
        new_kf_in_base_map->SetORBvocabulary(base_map_system_ptr->GetOrbVocabulary());
        new_kf_in_base_map->ReconstructKeyFrame(base_map_system_ptr->GetKeyFrameDatabase(), base_map_ptr);


    }


    cout << "Base map statistics after integration ...." << endl;
    cout << "Keyframes: " << base_map_system_ptr->GetAllKeyFrames().size () << ", map-points: " << base_map_system_ptr->GetAllMapPoints().size() << endl;

    // Now, we can finally integrate these keyframes and their map-points into the base map
    // The keyframes have everything but the map-points do not have their mnids

    return;
}



int main(int argc, char **argv) {
    cout << "CarMap cloud service initiated ...." << endl;

    // Parse the configuration file
    cv::FileStorage fSettings(pathToConfigFile, cv::FileStorage::READ);
    if (!fSettings.isOpened()) {
        cerr << "Failed to open config file " << pathToConfigFile << endl;
        return -1;
    }
    ReadAndParseConfigFile(fSettings);

    // Open the log file for logging end to end latency
    std::ofstream log_file (path_to_log_file, std::ofstream::out | std::ofstream::app);
    if (log_file.is_open() == false)
    {
        cerr << "Could not open log file at: " << path_to_log_file << endl;
        return -1;
    }
    else
        cout << "Opened log file at: " << path_to_log_file << endl;

    // Load and reconstruct the base map
    cout << "Path to base one: " << pathToMapOne << endl;
    InitStitchingModule(oneShotStitching);
    ORB_SLAM2::System *baseMap = InitBaseMap(pathToMapOne, visualizeMapOne, mapOneStr, reconstructBaseMap);// Initialize the base map
    PrintNumberOfMapElements(baseMap, mapOneStr); // print number of elements in the base map

    // Initialize the second map
    // Receive it over the network or read it from disk
    ORB_SLAM2::System *received_map_system = initialize_ORB_SLAM_for_received_map(pathToVocabFile, pathToSettingsFile);
    ORB_SLAM2::Map *map_to_stitch = new ORB_SLAM2::Map();
    /*
    if (receive_vehicle_map_over_network == false)
    {
        cout << "Reading the second map from disk ...." << endl;
        ORB_SLAM2::System *relocalizeMap = InitSecondMap(pathToMapTwo, visualizeMapTwo, mapTwoStr, reconstructLostMap);// Initialize the map to augment
        PrintNumberOfMapElements(relocalizeMap, mapTwoStr);
    }

    //else
    //{
*/
    // Wait for a connection from the vehicle node
    boost::asio::io_service io_service;
    cout << "Initializing boost tcp acceptor objects ...." << endl;
    boost::asio::ip::tcp::acceptor acceptor(
            io_service,
            boost::asio::ip::tcp::endpoint(
                    boost::asio::ip::address::from_string(cloud_service_ip_address),
                    cloud_service_port_number
            )
    );
    cout << "Awaiting connections ...." << endl;
    boost::asio::ip::tcp::socket carmap_socket(io_service);
    acceptor.accept(carmap_socket);
    std::cout << "Accepted connection from: " << carmap_socket.remote_endpoint() << std::endl;

    float totalData = 0;

    std::time_t previousTime;

    // Connection has been established
    // Receive data, stitch it and then send it back to the vehicle
    log_file << "Cloud_Counter,Vehicle_Map_Size,Reconstruction_Time,Stitching_Time,Integration_Time,Cloud_Map_KFs,Vehicle_Map_KFs,Stitched_Map_KFs,Cloud_to_Vehicle_Time,Cloud_to_Vehicle_Map_Size" << endl;

    int rx_counter = 0;
    int base_maps_kfs, stitched_map_kfs;
    size_t header;
    float tx_time, rx_time, stitching_time, tx_map_size, rx_map_size, reconstruction_time, integration_time;
    int rx_map_kfs;
    std::chrono::steady_clock::time_point rx_begin, rx_end, tx_begin, tx_end, stitch_begin, stitch_end, reconstruction_begin, reconstruction_end, integration_begin, integration_end;


    while (true)
    {
        cout << "Awaiting a new map segment ...." << endl;
        base_maps_kfs = baseMap->GetAllKeyFrames().size();

        // Receive the map from the vehicle
        boost::asio::streambuf buf;
        rx_begin = std::chrono::steady_clock::now();
        size_t recv_map_size_in_bytes = ReceiveMap (&carmap_socket, &buf);
        rx_end = std::chrono::steady_clock::now();
        // send back the same map to measure RTT
        float rtt_map_size = SendMap (&carmap_socket, &buf);
        rx_map_size = recv_map_size_in_bytes * BYTES_TO_MB;
        cout << "Received " << rx_map_size << ", and sent " << rtt_map_size << endl;
        cout << "Received map over the network ...." << rx_map_size << " mb" << endl;
        // deserialize
        {
            std::istream is(&buf);
            boost::archive::binary_iarchive ar(is);
            ar & map_to_stitch;
        }

        rx_map_kfs = map_to_stitch->GetAllKeyFrames().size();
        // Reconstruct the received map
        if (integrate_diff == true)
        {
            integration_begin = std::chrono::steady_clock::now ();
            IntegrateDiffInMap(baseMap, map_to_stitch);
            integration_end = std::chrono::steady_clock::now ();

            cout << "Integrated the diff into the base map ...." << endl;
            cout << "Not going to stitch because we have already integrated the differences ...." << endl;

            // Time required to do operations
            stitching_time = 0;
            reconstruction_time = 0;
            integration_time = (std::chrono::duration_cast<std::chrono::microseconds>(integration_end - integration_begin).count()) / 1000000.0f;

        }
        else
        {
            reconstruction_begin = std::chrono::steady_clock::now ();
            received_map_system->ReconstructReceivedMap(map_to_stitch);
            reconstruction_end = std::chrono::steady_clock::now ();
            cout << "Partially reconstructed the received map ...." << endl;
            stitch_begin = std::chrono::steady_clock::now();
            stitching_time = StitchOperation(baseMap, map_to_stitch);
            stitch_end = std::chrono::steady_clock::now();
            cout << "Stitched the partially reconstructed map ...." << endl;

            // Time required to do operations
            stitching_time = (std::chrono::duration_cast<std::chrono::microseconds>(stitch_end - stitch_begin).count()) / 1000000.0f;
            reconstruction_time = (std::chrono::duration_cast<std::chrono::microseconds>(reconstruction_end-reconstruction_begin).count () ) / 1000000.0f;
            integration_time = 0;

        }



        cout << "Serializing the same received map file ...." << endl;
        boost::asio::streambuf write_buf;
        SerializeMap (map_to_stitch, &write_buf);
        cout << "Sending the serialized received binary binary file to the vehicle ...." << endl;
        tx_begin = std::chrono::steady_clock::now();
        tx_map_size = SendMap (&carmap_socket, &write_buf);
        float temp_rx_size = ReceiveMap (&carmap_socket, &write_buf);
        tx_end = std::chrono::steady_clock::now();

        tx_time = (std::chrono::duration_cast<std::chrono::microseconds>(tx_end - tx_begin).count()) / 1000000.0f;
//        rx_time = (std::chrono::duration_cast<std::chrono::microseconds>(rx_end - rx_begin).count()) / 1000000.0f;

	    stitched_map_kfs = baseMap->GetAllKeyFrames().size();

        log_file << rx_counter++ << ","
//                 << rx_time << ","
                 << rx_map_size << ","
                 << reconstruction_time << ","
                 << stitching_time << ","
                 << integration_time << ","
                 << base_maps_kfs << ","
                 << rx_map_kfs << ","
                 << stitched_map_kfs << ","
                 << ( tx_time / 2.0f ) << ","
                 << tx_map_size << endl << std::flush;



        // Saving the map once before another stitch or integration operation
//        baseMap->SaveMap(pathToSaveMap);
        // Freeing map to stitch
        received_map_system->Reset();
        delete (map_to_stitch);

    }

    cout << "Stitch operation completed ....." << endl;
    PrintNumberOfMapElements(baseMap, mapOneStr);
//    shutDownORBSLAM(relocalizeMap, "", false, "");
    shutDownORBSLAM(baseMap, "", false, pathToSaveMap);

    cout << "Tearing down the connection" << endl;
    carmap_socket.close();
    io_service.stop();

    return 0;
}
