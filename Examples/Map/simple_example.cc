

// STD
#include <string>
#include <iostream>
#include <unistd.h>

// OpenCV
#include <opencv2/core/core.hpp>

// Pangolin
#include <pangolin/pangolin.h>

// Project
#include "Map.h"
#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"


class Viewer
{
public:
    Viewer(ORB_SLAM2::MapDrawer * drawer, const std::string & settings) :
        mpMapDrawer(drawer)
    {
        cv::FileStorage fSettings(settings, cv::FileStorage::READ);
        mViewpointX = fSettings["Viewer.ViewpointX"];
        mViewpointY = fSettings["Viewer.ViewpointY"];
        mViewpointZ = fSettings["Viewer.ViewpointZ"];
        mViewpointF = fSettings["Viewer.ViewpointF"];
    }

    int Run() const
    {
        int width = 720;
        int height = 480;
        pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",720, 480);

        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);
        // Issue specific OpenGl we might need
        glEnable (GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
        pangolin::Var<bool> menuExit("menu.Exit", false, false);

        // Define Camera Render Object (for view / scene browsing)
        pangolin::OpenGlRenderState s_cam(
                    pangolin::ProjectionMatrix(width, height, mViewpointF, mViewpointF, 512, 389, 0.1, 1000),
                    pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                    );

        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View& d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -width*1.0f/height)
                .SetHandler(new pangolin::Handler3D(s_cam));

        pangolin::OpenGlMatrix Twc;
        Twc.SetIdentity();

        bool running = true;

        while(running)
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
            s_cam.Follow(Twc);
            d_cam.Activate(s_cam);
            glClearColor(1.0f,1.0f,1.0f,1.0f);
            mpMapDrawer->DrawCurrentCamera(Twc);
            mpMapDrawer->DrawMapPoints();
            pangolin::FinishFrame();

            if (menuExit)
            {
                running = false;
            }

            usleep(30000);
        }

        return 0;
    }

protected:

    ORB_SLAM2::MapDrawer* mpMapDrawer;
    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

};



int main(int argc, char** argv)
{

    if (argc != 2)
    {
        std::cout << "simple_example <viewer_conf.yaml>" << std::endl;
        return 0;
    }

    int ret = -1;
    int size = 10;
    std::vector<ORB_SLAM2::MapPoint*> mapPoints(size, 0);

    //Create the Map
    {
        std::string settings(argv[1]);
        ORB_SLAM2::Map map;
        ORB_SLAM2::MapDrawer drawer(&map, settings);

        map.clear();

        // create some map points:
//        for (int i=0; i<size; i++)
//        {
//            ORB_SLAM2::KeyFrame * keyframe;
//            cv::Mat pos;
//            ORB_SLAM2::MapPoint * mapPoint = new ORB_SLAM2::MapPoint(pos, keyframe, &map);
//            map.AddMapPoint(mapPoint);
//        }

        Viewer viewer(&drawer, settings);
        ret = viewer.Run();
    }
    // destroy map points:
//    for (int i=0; i<mapPoints.size; i++)
//    {
//        delete mapPoints[i];
//    }

    return ret;

}
