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
    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11")
    add_definitions(-DCOMPILEDWITHC11)
    message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++0x")
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
SET(VERSION_REQ_CUDA "8.0")


find_package(CUDA QUIET REQUIRED)
set(CUDA_PROPAGATE_HOST_FLAGS OFF)
SET(CUDA_HOST_COMPILER /usr/bin/g++)
LIST(APPEND CUDA_NVCC_FLAGS "--compiler-options -fno-strict-aliasing -use_fast_math")

find_package(Curses REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
#find_package(ZED 2 REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)
find_package(CUDA ${VERSION_REQ_CUDA} REQUIRED)
find_package(PCL 1.9 REQUIRED)

cuda_include_directories(
        ${CUDA_TOOLKIT_ROOT_DIR}/samples/common/inc
)


include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/ORB_SLAM2
        ${PROJECT_SOURCE_DIR}/ORB_SLAM2/include
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
	#        ${ZED_INCLUDE_DIRS}
        ${GLEW_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${CURSES_INCLUDE_DIR}
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
link_libraries(${JSONCPP_LIBRARIES})



#link_directories(${ZED_LIBRARY_DIR})
link_directories(${GLEW_LIBRARY_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)
link_directories(${PCL_LIBRARY_DIRS})

FILE(GLOB_RECURSE SRC_FILES src/*.cpp)
FILE(GLOB_RECURSE HDR_FILES include/*.h)
FILE(GLOB_RECURSE ORB_SLAM2_SRC ORB_SLAM2/src/*.cc)


set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

cuda_add_library(${PROJECT_NAME} SHARED
        #${ORB_SLAM2_SRC}
        #${HDR_FILES}
        #${SRC_FILES}
        #src/MapSegmentUploader.cpp
        ORB_SLAM2/src/System.cc
        ORB_SLAM2/src/Tracking.cc
        ORB_SLAM2/src/LocalMapping.cc
        ORB_SLAM2/src/LoopClosing.cc
        ORB_SLAM2/src/ORBextractor.cc
        ORB_SLAM2/src/ORBmatcher.cc
        ORB_SLAM2/src/FrameDrawer.cc
        ORB_SLAM2/src/Converter.cc
        ORB_SLAM2/src/MapPoint.cc
        ORB_SLAM2/src/KeyFrame.cc
        ORB_SLAM2/src/KDTree.cpp
        ORB_SLAM2/src/Map.cc
        ORB_SLAM2/src/MapDrawer.cc
        ORB_SLAM2/src/Optimizer.cc
        ORB_SLAM2/src/PnPsolver.cc
        ORB_SLAM2/src/Frame.cc
        ORB_SLAM2/src/KeyFrameDatabase.cc
        ORB_SLAM2/src/Sim3Solver.cc
        ORB_SLAM2/src/Initializer.cc
        ORB_SLAM2/src/Viewer.cc
        ORB_SLAM2/src/SemanticSegmentor.cpp
	src/ZEDInitialization.cpp
	#        src/GLViewer.cpp
        )

add_definitions(${PCL_DEFINITIONS})

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/ORB_SLAM2/Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/ORB_SLAM2/Thirdparty/g2o/lib/libg2o.so
        ${SPECIAL_OS_LIBS}
	#        ${ZED_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARIES}
        ${GLEW_LIBRARIES}
        ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_npp_LIBRARY}
        ${PCL_LIBRARIES}
        ${CURSES_LIBRARIES}
        nvToolsExt
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


add_executable(main
        src/main.cc)
target_link_libraries(main ${PROJECT_NAME})


