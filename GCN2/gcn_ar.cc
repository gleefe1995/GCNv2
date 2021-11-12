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
using namespace std;

ORB_SLAM2::ViewerAR viewerAR;

// void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
//                 vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);


// class ImageGrabber
// {
// public:
//     ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

//     void GrabImage(const sensor_msgs::ImageConstPtr& msg);

//     ORB_SLAM2::System* mpSLAM;
// };


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
    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // viewerAR.SetSLAM(&SLAM);

    // ImageGrabber igb(&SLAM);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image and depthmap from file
        // imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        // imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
        im = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenames[ni] << endl;
            return 1;
        }

        // thread tViewer = thread(&ORB_SLAM2::ViewerAR::Run,&viewerAR);

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

        // SLAM.TrackRGBD(imRGB,imD,tframe);
        SLAM.TrackMonocular(im,tframe);


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

// void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
//                 vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
// {
//     ifstream fAssociation;
//     fAssociation.open(strAssociationFilename.c_str());
//     while(!fAssociation.eof())
//     {
//         string s;
//         getline(fAssociation,s);
//         if(!s.empty())
//         {
//             stringstream ss;
//             ss << s;
//             double t;
//             string sRGB, sD;
//             ss >> t;
//             vTimestamps.push_back(t);
//             ss >> sRGB;
//             vstrImageFilenamesRGB.push_back(sRGB);
//             ss >> t;
//             ss >> sD;
//             vstrImageFilenamesD.push_back(sD);

//         }
//     }
// }

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



// void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
// {
//     // Copy the ros image message to cv::Mat.
//     cv_bridge::CvImageConstPtr cv_ptr;
//     try
//     {
//         cv_ptr = cv_bridge::toCvShare(msg);
//     }
//     catch (cv_bridge::Exception& e)
//     {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//         return;
//     }
//     cv::Mat im = cv_ptr->image.clone();
//     cv::Mat imu;
//     cv::Mat Tcw = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
//     int state = mpSLAM->GetTrackingState();
//     vector<ORB_SLAM2::MapPoint*> vMPs = mpSLAM->GetTrackedMapPoints();
//     vector<cv::KeyPoint> vKeys = mpSLAM->GetTrackedKeyPointsUn();

//     cv::undistort(im,imu,K,DistCoef);

//     if(bRGB)
//         viewerAR.SetImagePose(imu,Tcw,state,vKeys,vMPs);
//     else
//     {
//         cv::cvtColor(imu,imu,CV_RGB2BGR);
//         viewerAR.SetImagePose(imu,Tcw,state,vKeys,vMPs);
//     }    
// }