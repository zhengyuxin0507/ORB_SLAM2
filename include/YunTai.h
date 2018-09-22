#ifndef YUNTAI_H
#define YUNTAI_H

#include "Tracking.h"
#include "tic_toc.h"

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

    /*bool mbStopRequested;
    bool mbStopped;
    bool mbFinishRequested;
    bool mbFinished;*/

private:

    bool Stop();

    bool isStopped();

    bool stopRequested();

    bool CheckFinish();

    void SetFinish();

    bool mbStopRequested;
    bool mbStopped;
    bool mbFinishRequested;
    bool mbFinished;

    std::mutex mMutexStop;
    std::mutex mMutexFinish;
    std::mutex mMutex;
};

}

#endif