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

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>
#include <math.h>

namespace ORB_SLAM2 {


    MapDrawer::MapDrawer(Map *pMap, const string &strSettingPath) : mpMap(pMap) {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
        mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
        mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
        mPointSize = fSettings["Viewer.PointSize"];
        mCameraSize = fSettings["Viewer.CameraSize"];
        mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

    }

    MapDrawer::MapDrawer (const string &strSettingPath)
    {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
        mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
        mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
        mPointSize = fSettings["Viewer.PointSize"];
        mCameraSize = fSettings["Viewer.CameraSize"];
        mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];
    }


    void MapDrawer::DrawMapPoints() {
        const vector<MapPoint *> &vpMPs = mpMap->GetAllMapPoints();
        const vector<MapPoint *> &vpRefMPs = mpMap->GetReferenceMapPoints();
//        cout << "Ill crash after this" << endl;

        set<MapPoint *> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

//        cout << "Yes crashed" << endl;
        if (vpMPs.empty())
            return;

//        cout << "Yes crashed again" << endl;


        glPointSize(mPointSize);
        glBegin(GL_POINTS);

        for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
            if (vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
                continue;

//            cout << "Viewer" << endl;



            cv::Scalar labelColor = vpMPs[i]->GetSemanticLabelColor();

//            if (vpMPs[i]->GetReferenceKeyFrame()->GetmnId() > 171)
//                glColor3f (1.0f, 0.0f, 0.0f);
//            else
//                glColor3f (0.0f, 1.0f, 0.0f);

            glColor3f( labelColor[0]/255.0 , labelColor[1]/255.0, labelColor[2]/255.0);

//            cout << vpMPs.at(i)->mpObservation.find(vpMPs.at(i)->GetReferenceKeyFrame())->second.Keypoint.pt.x
//                 << ", " << vpMPs.at(i)->mpObservation.find(vpMPs.at(i)->GetReferenceKeyFrame())->second.Keypoint.pt.y << endl;


            cv::Mat pos = vpMPs[i]->GetWorldPos();
            glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));

//            break;
        }
        glEnd();

//        cout << "Maybe here?" << endl;


        glPointSize(mPointSize);
        glBegin(GL_POINTS);


        glColor3f(1.0, 0.0, 0.0);

        for (set<MapPoint *>::iterator sit = spRefMPs.begin(), send = spRefMPs.end(); sit != send; sit++) {
            if ((*sit)->isBad())
                continue;
            cv::Mat pos = (*sit)->GetWorldPos();
            glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));

        }

        glEnd();
//        cout << "Didn't crash at all" << endl;

    }

    float get3Ddistance (struct pointIn3D firstPoint, struct pointIn3D secondPoint)
    {
        //return the 3d distance between two points
        float x = pow ( (firstPoint.x - secondPoint.x), 2.0);// ( x2 - x1 )^2
        float y = pow ( (firstPoint.y - secondPoint.y), 2.0);// ( y2 - y1 )^2
        float z = pow ( (firstPoint.z - secondPoint.z), 2.0);// ( z2 - z1 )^2

        return sqrt (x + y + z);
    }


    void MapDrawer::updatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr newPointCloud)
    {
        memberPointCloud = newPointCloud;
    }



    bool xFilter (float x)
    {
        if (x > 1.25f && x < 2.0)
            return false;
        else if (x > 4.0f && x < 5.0f)
            return false;
        else if (x < -3.0f && x > -4.0f)
            return false;
        else
            return true;


    }

    void MapDrawer::DrawMapPointsOffline(pcl::PointCloud <pcl::PointXYZ>::Ptr cloudPtr, bool isFiltered, unsigned int followerView) {


        static int counter = 0;
        if (cloudPtr->points.size() == 0) {
            std::cout << "No points in file" << endl;
            return;
        } else;
            //cout << "Size = " << cloudPtr->points.size() << endl;

        glPointSize(5);
        glBegin(GL_POINTS);
        // (R, G, B)
        GLfloat first32 [3] = {255.0f, 0, 0};//Red color
        GLfloat second32 [3] = {0.0f, 255.0f, 0.0f};
        GLfloat third32 [3] = {0.0f, 0.0f, 255.0f};//Blue color

        const int ArraySize = 3;
        float pos [ArraySize] = {0};
        int distance = 0;
        int colorCounter = 0;
        int pointerCounter = 0;
        static bool displayOnce = false;
        float ymin = 100;

        struct pointIn3D origin;
        origin.x = origin.y = origin.z = 0;

         displayOnce = true;
        if (isFiltered)
            colorCounter = 2;

//        glColor3f(counter++/1000.0,0.25f,0.0);
//
//        cout << counter / 1000.0 << endl;
//        if (counter == 1000)
//            counter = 0;
        for (size_t pointCounter = 0, pointCloudEnd = cloudPtr->points.size(); pointCounter < pointCloudEnd; pointCounter++)
        {

            struct pointIn3D currentPoint;
            currentPoint.x = cloudPtr->points[pointCounter].x;
            currentPoint.y = cloudPtr->points[pointCounter].y;
            currentPoint.z = cloudPtr->points[pointCounter].z;


            float distance = get3Ddistance(currentPoint, origin);



            float blueColor = ( (750.0 - distance) / 750.0);
//            cout << blueColor << endl;
            //int redColor = 255 - 3dDistance / 100;
            bool skip = false;
////
//            if (followerView != 0) {
//                if (pointCounter < followerView)
//                    glColor3f(third32[0], third32[1], third32[2]);
//                else
//                    glColor3f(first32[0], first32[1], first32[2]);
//            }
//            else if (isFiltered)
//                glColor3f(third32[0], third32[1], third32[2]);
//            else
//                glColor3f(first32[0], first32[1], first32[2]);





            glColor3f(1-blueColor, 0.25f, blueColor);
//

                if (displayOnce == false && skip == false) {
                    if (ymin > cloudPtr->points[pointCounter].y)
                        ymin = cloudPtr->points[pointCounter].y;
                    //cout << pointerCounter++ << " : " << cloudPtr->points[pointCounter].x << " , "
                      //   << cloudPtr->points[pointCounter].y << " , " << cloudPtr->points[pointCounter].z << endl;

                }
                if (skip == false)
                   // if (xFilter(cloudPtr->points[pointCounter].x))
                glVertex3f(cloudPtr->points[pointCounter].x, cloudPtr->points[pointCounter].y,
                           cloudPtr->points[pointCounter].z);

        }
//        if (displayOnce == false)
//            cout << "Minimum y = " << ymin << " + 4.4704 = " << ymin + 4.4704f  << endl;
        displayOnce = true;
        glEnd();

//        cout << "Exitting" << endl;

        //glEnd();
    }



    void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph) {
        const float &w = mKeyFrameSize;
        const float h = w * 0.75;
        const float z = w * 0.6;

        const vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();

        if (bDrawKF) {
            for (size_t i = 0; i < vpKFs.size(); i++) {
                KeyFrame *pKF = vpKFs[i];
                cv::Mat Twc = pKF->GetPoseInverse().t();

                glPushMatrix();

                glMultMatrixf(Twc.ptr<GLfloat>(0));

                glLineWidth(mKeyFrameLineWidth);
//                cout << pKF->GetmnId() << endl;
//                if (pKF->GetmnId() > 171l)
//                    glColor3f(1.0f, 0.0f, 0.0f);
//                else
//                    glColor3f(0.0f, 1.0f, 0.0f);

//                if (i < vpKFs.size() / 2)
                    glColor3f(0.0f, 0.0f, 1.0f);
//                else
//                    glColor3f(1.0f, 0.0f, 0.0f);
                glBegin(GL_LINES);
                glVertex3f(0, 0, 0);
                glVertex3f(w, h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(w, -h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(-w, -h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(-w, h, z);

                glVertex3f(w, h, z);
                glVertex3f(w, -h, z);

                glVertex3f(-w, h, z);
                glVertex3f(-w, -h, z);

                glVertex3f(-w, h, z);
                glVertex3f(w, h, z);

                glVertex3f(-w, -h, z);
                glVertex3f(w, -h, z);
                glEnd();

                glPopMatrix();
            }
        }

        if (bDrawGraph) {
            glLineWidth(mGraphLineWidth);
            glColor4f(0.0f, 1.0f, 0.0f, 0.6f);
            glBegin(GL_LINES);

            for (size_t i = 0; i < vpKFs.size(); i++) {
                // Covisibility Graph
                const vector<KeyFrame *> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
                cv::Mat Ow = vpKFs[i]->GetCameraCenter();
                if (!vCovKFs.empty()) {
                    for (vector<KeyFrame *>::const_iterator vit = vCovKFs.begin(), vend = vCovKFs.end();
                         vit != vend; vit++) {
                        if ((*vit)->mnId < vpKFs[i]->mnId)
                            continue;
                        cv::Mat Ow2 = (*vit)->GetCameraCenter();
                        glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
                        glVertex3f(Ow2.at<float>(0), Ow2.at<float>(1), Ow2.at<float>(2));
                    }
                }

                // Spanning tree
                KeyFrame *pParent = vpKFs[i]->GetParent();
                if (pParent) {
                    cv::Mat Owp = pParent->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
                    glVertex3f(Owp.at<float>(0), Owp.at<float>(1), Owp.at<float>(2));
                }

                // Loops
                set<KeyFrame *> sLoopKFs = vpKFs[i]->GetLoopEdges();
                for (set<KeyFrame *>::iterator sit = sLoopKFs.begin(), send = sLoopKFs.end(); sit != send; sit++) {
                    if ((*sit)->mnId < vpKFs[i]->mnId)
                        continue;
                    cv::Mat Owl = (*sit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
                    glVertex3f(Owl.at<float>(0), Owl.at<float>(1), Owl.at<float>(2));
                }
            }

            glEnd();
        }
    }

    void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc) {
        const float &w = mCameraSize;
        const float h = w * 0.75;
        const float z = w * 0.6;

        glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

        glLineWidth(mCameraLineWidth);
        glColor3f(0.0f, 1.0f, 0.0f);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(w, h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, h, z);

        glVertex3f(w, h, z);
        glVertex3f(w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(-w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);

        glVertex3f(-w, -h, z);
        glVertex3f(w, -h, z);
        glEnd();

        glPopMatrix();
    }


    void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw) {
//        cout << "Set camera pose" << endl;
//        cout << Tcw << endl;
        unique_lock<mutex> lock(mMutexCamera);
        mCameraPose = Tcw.clone();
    }

    void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, bool isRunOffline = false) {

        if (isRunOffline == false) {

            if (!mCameraPose.empty()) {
                cv::Mat Rwc(3, 3, CV_32F);
                cv::Mat twc(3, 1, CV_32F);
                {
                    unique_lock<mutex> lock(mMutexCamera);
                    Rwc = mCameraPose.rowRange(0, 3).colRange(0, 3).t();
                    twc = -Rwc * mCameraPose.rowRange(0, 3).col(3);
                }

                M.m[0] = Rwc.at<float>(0, 0);
                M.m[1] = Rwc.at<float>(1, 0);
                M.m[2] = Rwc.at<float>(2, 0);
                M.m[3] = 0.0;

                M.m[4] = Rwc.at<float>(0, 1);
                M.m[5] = Rwc.at<float>(1, 1);
                M.m[6] = Rwc.at<float>(2, 1);
                M.m[7] = 0.0;

                M.m[8] = Rwc.at<float>(0, 2);
                M.m[9] = Rwc.at<float>(1, 2);
                M.m[10] = Rwc.at<float>(2, 2);
                M.m[11] = 0.0;

                M.m[12] = twc.at<float>(0);
                M.m[13] = twc.at<float>(1);
                M.m[14] = twc.at<float>(2);
                M.m[15] = 1.0;
            } else {
//                cout << "No camera pose" << endl;

                M.SetIdentity();
            }
        }
        else
            M.SetIdentity();
    }

} //namespace ORB_SLAM
