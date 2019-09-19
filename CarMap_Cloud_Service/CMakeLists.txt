cmake_minimum_required(VERSION 2.8)
project(Crowdsourced_HD_Map)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Debug)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    #set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11")
    add_definitions(-DCOMPILEDWITHC11)
    message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    #set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else()
    message(FATACHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)L_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.4.0 REQUIRED)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif()
endif()

SET(EXECUTABLE_OUTPUT_PATH ".")
SET(SPECIAL_OS_LIBS "")


find_package(Curses REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/ORB_SLAM2
        ${PROJECT_SOURCE_DIR}/ORB_SLAM2/include
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        ${CURSES_INCLUDE_DIR}
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
link_libraries(${JSONCPP_LIBRARIES})

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)
link_directories(${PCL_LIBRARY_DIRS})

FILE(GLOB_RECURSE SRC_FILES src/*.cpp)
FILE(GLOB_RECURSE HDR_FILES include/*.h)
FILE(GLOB_RECURSE ORB_SLAM2_SRC ORB_SLAM2/src/*.cc)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
        src/MapSegmentUploader.cpp
        ORB_SLAM2/src/System.cc
        ORB_SLAM2/src/Tracking.cc
        ORB_SLAM2/src/LocalMapping.cc
        ORB_SLAM2/src/LoopClosing.cc
        ORB_SLAM2/src/ORBextractor.cc
        ORB_SLAM2/src/ORBmatcher.cc
        ORB_SLAM2/src/Converter.cc
        ORB_SLAM2/src/MapPoint.cc
        ORB_SLAM2/src/KeyFrame.cc
        ORB_SLAM2/src/KDTree.cpp
        ORB_SLAM2/src/Map.cc
        ORB_SLAM2/src/Optimizer.cc
        ORB_SLAM2/src/PnPsolver.cc
        ORB_SLAM2/src/Frame.cc
        ORB_SLAM2/src/KeyFrameDatabase.cc
        ORB_SLAM2/src/Sim3Solver.cc
        ORB_SLAM2/src/Initializer.cc
        ORB_SLAM2/src/SemanticSegmentor.cpp
        src/ZEDInitialization.cpp
        )

add_definitions(${PCL_DEFINITIONS})

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/ORB_SLAM2/Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/ORB_SLAM2/Thirdparty/g2o/lib/libg2o.so
        ${SPECIAL_OS_LIBS}
        ${PCL_LIBRARIES}
        ${CURSES_LIBRARIES}
        )

message(STATUS "Compile With map save/load function")
find_library(BOOST_SERIALIZATION boost_serialization)
if (NOT BOOST_SERIALIZATION)
    message(FATAL_ERROR "Can't find libboost_serialization")
endif()

find_library(BOOST_SYSTEM boost_system)
if (NOT BOOST_SYSTEM)
    message(FATAL_ERROR "Can't find libboost_serialization")
endif()
target_link_libraries(${PROJECT_NAME} ${BOOST_SYSTEM})


add_executable(server_stitch src/server_stitch.cpp include/GlobalDefintions.h include/ZEDInitialization.h)
target_link_libraries (server_stitch ${PROJECT_NAME})
