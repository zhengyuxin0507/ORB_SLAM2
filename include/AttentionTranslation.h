#ifndef ATTENTIONTRANSLATION_H
#define ATTENTIONTRANSLATION_H

#include "Tracking.h"
#include "YunTai.h"
#include "Map.h"
#include "tic_toc.h"

#include <mutex>

namespace ORB_SLAM2
{

    class YunTai;
    class Tracking;
    class MapDrawer;

class AttentionTranslation
{
public:
    AttentionTranslation(Map *pMap, MapDrawer* pMapDrawer);

    // Main function
    void Run();

    void SetTracker(Tracking *pTracker);

    void SetYunTai(YunTai *pYunTai);

    bool GetStatus();

    void GetCameraPose();

    void GetYunTaiPose(const cv::Mat &Tcw, cv::Mat &Tyw);

    int GetMapPointsInView(const float theta, const set< pair<float, float> > &sMapPointProject);

    void DrawFrame();

    cv::Mat mShowMat;

private:

    Tracking* mpTracker;
    YunTai* mpYunTai;
    Map* mpMap;  //Map
    MapDrawer* mpMapDrawer;

    float mTheta;   //the rotation(rad) between YunTai and Camera in y axis
    float mThetaStep;
    float mWindowWidth;
    float mWindowHeight;

    long mLastFrameNum;
    long mFrameNum;

    std::vector<int> mvMapPointHist;
    std::vector< pair<float, float> > mvMapPointProject;

    cv::Mat mTcw;

    std::mutex mMutexTcw;
    std::mutex mMutexbOK;

};

}

#endif