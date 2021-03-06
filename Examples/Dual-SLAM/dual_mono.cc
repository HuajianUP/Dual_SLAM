/**
* This is a modified version of mono_kitti.cpp of ORB-SLAM2.
*
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
*/

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>
#include <iostream>
#include <string>

#include <sstream>
#include <dirent.h>

#include<opencv2/core/core.hpp>

#include"System.h"

using namespace std;

int getdir (const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./dual_mono path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }


    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;

    getdir(string(argv[3]), vstrImageFilenames, vTimestamps);
    //LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

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
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T = 0.03;
        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }
    cout<<endl<<endl<<"press any key to shutdown"<<endl;
    getchar();
    // Stop all threads
    SLAM.Shutdown();

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
    cout<<endl<<endl<<"press any key to shutdown"<<endl;
    getchar();
    // Save camera trajectory
    SLAM.getKeyframe("KeyFrame_Mono.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_Mono.txt");    

    return 0;
}


int getdir (const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    std::string dir=strPathToSequence;
    DIR *dp;
    struct dirent *dirp;
    std::vector<std::string> files;
    if((dp  = opendir(dir.c_str())) == NULL)
    {
        return -1;
    }
    while ((dirp = readdir(dp)) != NULL) {
        std::string name = std::string(dirp->d_name);

        if(name != "." && name != "..")
            files.push_back(name);
    }
    closedir(dp);

    std::sort(files.begin(), files.end());

    if(dir.at( dir.length() - 1 ) != '/') 
        dir = dir+"/";

    for(unsigned int i=0;i<files.size();i++)
    {
        if(files[i].at(0) != '/')
        {
            vTimestamps.push_back(i);
            vstrImageFilenames.push_back(dir + files[i]);
        }

    }
    cout<<"get files"<<endl;
    return files.size();

}
