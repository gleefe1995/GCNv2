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

#include <torch/script.h> // One-stop header.
#include <torch/torch.h>

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <System.h>

#include "ViewerAR.h"

#include "pointcloud_2_depth.h"

using namespace std;

ORB_SLAM2::ViewerAR viewerAR;
bool bRGB = true;

cv::Mat K;
cv::Mat DistCoef;
// void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
//                 vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

void LoadCameraPose(const cv::Mat &Tcw);

void DrawCube(const float &size,const float x, const float y, const float z);

void DrawTrackedPoints(const std::vector<cv::KeyPoint> &vKeys, const std::vector<ORB_SLAM2::MapPoint *> &vMPs, cv::Mat &im);




int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./rgbd_gcn path_to_vocabulary path_to_settings path_to_sequence path_to_association" << endl;
        return 1;
    }
    

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    string strFile = string(argv[3])+"/rgb.txt";
    LoadImages(strFile, vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();
    cv::Mat im2;
    im2 = cv::imread(string(argv[3])+"/"+vstrImageFilenames[0],CV_LOAD_IMAGE_UNCHANGED);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    
    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    

    cv::FileStorage fSettings(argv[2], cv::FileStorage::READ);
    bRGB = static_cast<bool>((int)fSettings["Camera.RGB"]);
    float fps = fSettings["Camera.fps"];
    // viewerAR.SetFPS(fps);

    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;

    DistCoef = cv::Mat::zeros(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }

    std::string filename = "/home/gleefe/D3Feat.pytorch/test_pcd.pcd";

    
    // std::shared_ptr<open3d::geomtery::PointCloud> test_ply = std::make_shared<open3d::geometry::PointCloud>();
    
    
    std::vector<cv::Point3f> object_points;
    // auto test_ply = std::make_shared<open3d::geometry::PointCloud>();
    // open3d::geometry::PointCloud test_ply;
    // object_points.reserve(pt_cloud.size());
    
    pc2d::read_objectpoint(object_points,filename);


    // std::cout<<object_points.size()<<std::endl;


    int w,h,wui;
    w = im2.cols;
    h = im2.rows;
    wui=200;
    pangolin::CreateWindowAndBind("Viewer",w+wui,h);
    cout<<"Create window"<<endl;

    glEnable(GL_DEPTH_TEST);
    glEnable (GL_BLEND);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(wui));
    pangolin::Var<bool> menu_detectplane("menu.Insert Cube",false,false);
    pangolin::Var<bool> menu_clear("menu.Clear All",false,false);
    pangolin::Var<bool> menu_drawim("menu.Draw Image",true,true);
    pangolin::Var<bool> menu_drawcube("menu.Draw Cube",true,true);
    pangolin::Var<float> menu_cubesize("menu. Cube Size",0.05,0.01,0.3);
    pangolin::Var<bool> menu_drawgrid("menu.Draw Grid",true,true);
    pangolin::Var<int> menu_ngrid("menu. Grid Elements",3,1,10);
    pangolin::Var<float> menu_sizegrid("menu. Element Size",0.05,0.01,0.3);
    pangolin::Var<bool> menu_drawpoints("menu.Draw Points",false,true);



    pangolin::View& d_image = pangolin::Display("image")
        .SetBounds(0,1.0f,pangolin::Attach::Pix(wui),1.0f,(float)w/h)
        .SetLock(pangolin::LockLeft, pangolin::LockTop);


    pangolin::GlTexture imageTexture(w,h,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);

    pangolin::OpenGlMatrixSpec P = pangolin::ProjectionMatrixRDF_TopLeft(w,h,fx,fy,cx,cy,0.001,1000);

    vector<ORB_SLAM2::Plane*> vpPlane;


    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image and depthmap from file
        // imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        // imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
        im = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];
        // std::cout<<"imread"<<std::endl;
        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenames[ni] << endl;
            return 1;
        }
        // std::cout<<"Tcw"<<std::endl;
        cv::Mat Tcw = SLAM.TrackMonocular(im,tframe);
        // std::cout<<"Tcw"<<std::endl;
            int state = SLAM.GetTrackingState();
            vector<ORB_SLAM2::MapPoint*> vMPs = SLAM.GetTrackedMapPoints();
            vector<cv::KeyPoint> vKeys = SLAM.GetTrackedKeyPointsUn();

        // if(imRGB.empty())
        // {
        //     cerr << endl << "Failed to load image at: "
        //          << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
        //     return 1;
        // }


#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
        // std::cout<<"trackmonocular"<<std::endl;
        // SLAM.TrackRGBD(imRGB,imD,tframe);
        // SLAM.TrackMonocular(im,tframe);
        
        // std::cout<<"depth image"<<std::endl;
        cv::Mat depth_image = pc2d::depth_image(object_points,Tcw,K);

        cv::imshow("vis", depth_image);
        cv::waitKey(10);

        {
            // GetImagePose(im,Tcw,status,vKeys,vMPs);

            d_image.Activate();
            glColor3f(1.0,1.0,1.0);


            // cout<<"current cube number: "<<vpPlane.size()<<endl;

            if(menu_drawpoints)
                DrawTrackedPoints(vKeys,vMPs,im);
            //Draw image
            if(!im.empty())
            {
                imageTexture.Upload(im.data,GL_RGB,GL_UNSIGNED_BYTE);
                imageTexture.RenderToViewportFlipY();
            }

            glClear(GL_DEPTH_BUFFER_BIT);

            // Load camera projection
            glMatrixMode(GL_PROJECTION);
            P.Load();

            glMatrixMode(GL_MODELVIEW);
            
            // Load camera pose
            LoadCameraPose(Tcw);



            // viewerAR.DrawPlane(menu_ngrid,menu_sizegrid);


            if(menu_clear)
            {
                if(!vpPlane.empty())
                {
                    for(size_t i=0; i<vpPlane.size(); i++)
                    {
                        delete vpPlane[i];
                    }
                    vpPlane.clear();
                    cout << "All cubes erased!" << endl;
                }
                menu_clear = false;
            }

            //draw virtual things
            if(menu_detectplane)
            {
                ORB_SLAM2::Plane* pPlane = viewerAR.DetectPlane(Tcw,vMPs,50);
                if(pPlane)
                {
                    cout << "New virtual cube inserted!" << endl;
                    vpPlane.push_back(pPlane);
                }
                else
                {
                    cout << "No plane detected. Point the camera to a planar region." << endl;
                }
                menu_detectplane = false;
            }

            if(!vpPlane.empty())
            {
                // Recompute plane if there has been a loop closure or global BA
                // In localization mode, map is not updated so we do not need to recompute
                bool bRecompute = false;
                // if(!bLocalizationMode)
                // {
                //     if(mpSystem->MapChanged())
                //     {
                //         cout << "Map changed. All virtual elements are recomputed!" << endl;
                //         bRecompute = true;
                //     }
                // }

                if(SLAM.MapChanged())
                    {
                        cout << "Map changed. All virtual elements are recomputed!" << endl;
                        bRecompute = true;
                    }



                for(size_t i=0; i<vpPlane.size(); i++)
                {
                    ORB_SLAM2::Plane* pPlane = vpPlane[i];

                    if(pPlane)
                    {
                        if(bRecompute)
                        {
                            pPlane->Recompute();
                        }
                        glPushMatrix();
                        pPlane->glTpw.Multiply();

                        // Draw cube
                        if(menu_drawcube)
                        {
                            viewerAR.DrawCube(menu_cubesize);
                        }

                        // Draw grid plane
                        if(menu_drawgrid)
                        {
                            viewerAR.DrawPlane(menu_ngrid,menu_sizegrid);
                        }

                        glPopMatrix();
                    }
                }
            }



            pangolin::FinishFrame();
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    std::cout << "Finished!" << std::endl;

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    // Stop all threads
    SLAM.Shutdown();
   

    return 0;
}



void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}



void LoadCameraPose(const cv::Mat &Tcw)
{
    if(!Tcw.empty())
    {
        pangolin::OpenGlMatrix M;

        M.m[0] = Tcw.at<float>(0,0);
        M.m[1] = Tcw.at<float>(1,0);
        M.m[2] = Tcw.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Tcw.at<float>(0,1);
        M.m[5] = Tcw.at<float>(1,1);
        M.m[6] = Tcw.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Tcw.at<float>(0,2);
        M.m[9] = Tcw.at<float>(1,2);
        M.m[10] = Tcw.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = Tcw.at<float>(0,3);
        M.m[13] = Tcw.at<float>(1,3);
        M.m[14] = Tcw.at<float>(2,3);
        M.m[15]  = 1.0;

        M.Load();
    }
}



void DrawCube(const float &size,const float x, const float y, const float z)
{
    pangolin::OpenGlMatrix M = pangolin::OpenGlMatrix::Translate(-x,-size-y,-z);
    glPushMatrix();
    M.Multiply();
    pangolin::glDrawColouredCube(-size,size);
    glPopMatrix();
}





void DrawTrackedPoints(const std::vector<cv::KeyPoint> &vKeys, const std::vector<ORB_SLAM2::MapPoint *> &vMPs, cv::Mat &im)
{
    const int N = vKeys.size();
    // cout<<"DrawTrackPoint"<<endl;
    // cout<<"number of keypoint: "<<N<<endl;
    // cout<<"number of map oint: "<<vMPs.size()<<endl;

    for(int i=0; i<N; i++)
    {
        if(vMPs[i])
        {
            cv::circle(im,vKeys[i].pt,1,cv::Scalar(0,255,0),-1);
        }
    }
}