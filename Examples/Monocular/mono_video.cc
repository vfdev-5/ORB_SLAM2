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


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_video path_to_vocabulary path_to_settings path_to_video_file" << endl;
        return 1;
    }


    // Retrieve paths to images
    string videoFile = string(argv[3]);

    cv::VideoCapture capture(videoFile);
    if (!capture.isOpened())
    {
        cerr << "Failed to open video file" << endl;
        return -1;
    }

//    double fps = capture.get(CV_CAP_PROP_FPS);
    double fps = 30.0;
    cout << "Video fps : " << fps << endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

    cout << endl << "-------" << endl;
    cout << "Start processing video file ..." << endl;

    //    string window_name = "video | q or esc to quit";
    //    cout << "press space to save a picture. q or esc to quit" << endl;
    //    cv::namedWindow(window_name); //resizable window;

    cv::Mat frame;

    int waitTimeMs = static_cast<int>(1000.0 / fps);


    for (;;) {
        capture >> frame;
        if (frame.empty())
            break;

        double tframe = capture.get(CV_CAP_PROP_POS_MSEC) * 0.001;
        cout << "Frame time : " << tframe << " | ";

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        cv::Mat camPose = SLAM.TrackMonocular(frame,tframe);
        if (camPose.empty())
        {
            // Failed to track
        }


#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrackMs= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count() * 1000.0;

        // Wait to load the next frame
        cout << "ttrackMs : " << ttrackMs << " | waitTimeMs : " << waitTimeMs << endl;
        if(ttrackMs<waitTimeMs)
        {
//            char key = (char) cv::waitKey((waitTimeMs-ttrackMs)); //delay N millis, usually long enough to display and capture input
//            switch (key) {
//            case 'q':
//            case 'Q':
//            case 27: //escape key
//                return 0;
//            default:
//                break;
//            }
            usleep((waitTimeMs-ttrackMs)*1e3);
        }
    }

    cout << "Video is ended" << endl;

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    capture.release();

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

