#ifndef YUNTAI_H
#define YUNTAI_H

#include "Tracking.h"
#include "tic_toc.h"
#include "serial.h"

#include <mutex>

namespace ORB_SLAM2
{

class YunTai
{
public:
    YunTai();

    //~YunTai();

    int GetMapPointsInView(const float theta, const set< pair<float, float> > &sMapPointProject);

    //Get rotation matrix between YunTai and Camera
    void GetYunTaiPose(const cv::Mat &Tcw, cv::Mat &Tyw);

    //Update YunTai pose from tracking thread
    void UpdateYunTaiPose(const float theta);

    // Main function
    void Run();

    void RequestFinish();

    bool isFinished();

    //the rotation(rad) between YunTai and Camera in y axis
    float mTheta;

    float mThetaStep;
    float mWindowWidth;
    float mWindowHeight;

    std::vector<int> mvMapPointHist;
    std::vector< pair<float, float> > mvMapPointProject;

    //rotation matrix between YunTai and world
    cv::Mat mTyw;

private:

    bool CheckFinish();

    void SetFinish();

    void TurnLeft(const float v);

    void TurnRight(const float v);

    void Stop();

    void GetYaw(int &yaw);

    bool mbFinishRequested;
    bool mbFinished;

    int fd;

    std::mutex mMutexFinish;
    std::mutex mMutex;
};

}

#endif