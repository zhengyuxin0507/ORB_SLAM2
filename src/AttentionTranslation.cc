#include "AttentionTranslation.h"

namespace ORB_SLAM2
{

AttentionTranslation::AttentionTranslation(Map *pMap, MapDrawer* pMapDrawer):
mTheta(0), mShowMat(cv::Mat::zeros(320,640,CV_8UC1)), mpMap(pMap),mpMapDrawer(pMapDrawer), mThetaStep(0.05), mWindowWidth(0.84), mWindowHeight(0.61)
{
    //TODO:
}

//Main function
void AttentionTranslation::Run()
{
    cout << endl <<"Attention Transalation!!!!" << endl;

    while(1)
    {
        bool bOK = GetStatus();

        cv::Mat Tyw;
        if(bOK)
        {
            TicToc t_AT;
            bOK = 0;
            GetCameraPose();
            //if(mTcw.empty() || mFrameNum == mLastFrameNum )
            if(mTcw.empty())
            {
                usleep(30000);
                continue;
            }
            GetYunTaiPose(mTcw, Tyw);
            mpMapDrawer->SetCurrenYunTaiPose(Tyw);

            DrawFrame();

            cout << "Attention Translation time consuming: " << t_AT.toc() << endl;
        }
        else
        {
            //DrawFrame();
            usleep(5000);
        }

    }
}

void AttentionTranslation::DrawFrame()
{
    cv::Mat temp(320, 640, CV_8UC1, 255);
    for(size_t i = 0; i < mvMapPointProject.size(); i++)
    {
        int idx = round((mvMapPointProject[i].first + M_PI) * 100);
        int idy = round((-mvMapPointProject[i].second + M_PI/2) * 100);

        cv::circle(temp, cvPoint(idx, idy), 2, 0, 1);
    }
    for(int i = 0; i < 640; i++)
    temp.at<uchar>(160, i) = 0;
    for(int j = 0; j < 320; j++)
    {
        temp.at<uchar>(j, 160) = 0;
        temp.at<uchar>(j, 320) = 0;
        temp.at<uchar>(j, 480) = 0;
    }

    mShowMat = temp;
}

void AttentionTranslation::GetYunTaiPose_second(const cv::Mat &Tcw, cv::Mat &Tyw)
{
    TicToc t_m;

    const cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    const cv::Mat tcw = Tcw.rowRange(0,3).col(3);

    mvMapPointProject.clear();

    vector<MapPoint*> vMapPoints = mpMap->GetAllMapPoints();

    for(size_t i = 0; i < vMapPoints.size(); i++)
    {
        MapPoint* pMP = vMapPoints[i];
        if(pMP && !pMP->isBad())
        {
            cv::Mat x3Dw = pMP->GetWorldPos();
            cv::Mat x3Dc = Rcw*x3Dw+tcw;

            const float xc = x3Dc.at<float>(0);
            const float yc = x3Dc.at<float>(1);
            const float zc = x3Dc.at<float>(2);

            if(fabs(zc) > 5)
                continue;

            float theta = atan2(xc, zc);
            float gama = atan(-yc/sqrt(zc*zc+xc*xc));
            
            mvMapPointProject.push_back(make_pair(theta, gama));
        }
    }

    set< pair<float, float> > sMapPointProject;
    int iMapPointProjectNum = 0;
    for(size_t i = 0; i < mvMapPointProject.size(); i++)
    {
        if(mvMapPointProject[i].first > (-1 - mWindowWidth/2) && mvMapPointProject[i].first < (1 + mWindowWidth/2)) && mvMapPointProject[i].second > -mWindowHeight/2 && mvMapPointProject[i].second < mWindowHeight/2)
        {
            sMapPointProject.insert(make_pair(mvMapPointProject[i].first, mvMapPointProject[i].second));
            nMapPointProjectNum++;
        }
    }

    cout << "Point Project time consume: " << t_m.toc() << endl;

    vector< pair<int, float> > vScore;
    for(float i = -1; i <= 1; i+=0.2)
    {
        pair<int, float> temp = GetScore(i-mWindowWidth/2, i+mWindowWidth/2, sMapPointProject);
        vScore
    }

}

pair<int, float> AttentionTranslation::GetScore(const float lb, const float ub, const set< pair<float, float> > &sMapPointProject)
{
    float ThetaCenter = 0;
    float GamaCenter = 0;
    vector< pair<float, float> > vTemp;

    set< pair<float, float> >::iterator it;
    for(it = sMapPointProject.lower_bound(make_pair(lb, -4.0f)); it != sMapPointProject.lower_bound(make_pair(ub, -4.0f)); it++)
    {
        if(fabs(it->second) > mWindowHeight/2)
            continue;

        ThetaCenter += it->first;
        GamaCenter += it->second;
        vTemp.push_back(make_pair(it->first, it->second));
    }

    ThetaCenter /= vTemp.size();
    GamaCenter /= vTemp.size();

    float var = 0;
    for(int i = 0; i < vTemp.size(); i++)
    {
        var += sqrt((vTemp[i]->first-ThetaCenter)*(vTemp[i]->first-ThetaCenter) + (vTemp[i]->second-GamaCenter)*(vTemp[i]->second-GamaCenter)) 
    }
    var /= vTemp.size();

    return make_pair(vTemp.size(), var);
}

void AttentionTranslation::GetYunTaiPose(const cv::Mat &Tcw, cv::Mat &Tyw)
{
    TicToc t_m;

    const cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    const cv::Mat tcw = Tcw.rowRange(0,3).col(3);

    mvMapPointProject.clear();

    vector<MapPoint*> vMapPoints = mpMap->GetAllMapPoints();

    for(size_t i = 0; i < vMapPoints.size(); i++)
    {
        MapPoint* pMP = vMapPoints[i];
        if(pMP && !pMP->isBad())
        {
            cv::Mat x3Dw = pMP->GetWorldPos();
            cv::Mat x3Dc = Rcw*x3Dw+tcw;

            const float xc = x3Dc.at<float>(0);
            const float yc = x3Dc.at<float>(1);
            const float zc = x3Dc.at<float>(2);

            if(fabs(zc) > 5)
                continue;

            float xc2 = xc*xc;
            float zc2 = zc*zc;
            float theta = atan2(xc, zc);
            float gama = atan(-yc/sqrt(xc2+zc2));
            
            mvMapPointProject.push_back(make_pair(theta, gama));
        }
    }


    set< pair<float, float> > sMapPointProject;
    for(size_t i = 0; i < mvMapPointProject.size(); i++)
    {
        sMapPointProject.insert(make_pair(mvMapPointProject[i].first, mvMapPointProject[i].second));
    }

    cout << "Point Project time consume: " << t_m.toc() << endl;

    //Search right
    float CurrentThetaRight = 0;
    int CurrentMPRight = GetMapPointsInView(CurrentThetaRight, sMapPointProject);
    float NextThetaRight = CurrentThetaRight + mThetaStep;
    if(NextThetaRight > M_PI)
        NextThetaRight -= 2*M_PI;
    int NextMPRight;
    int i = 0;
    while(1)
    {
        i++;
        NextMPRight = GetMapPointsInView(NextThetaRight, sMapPointProject);
        if(NextMPRight >= CurrentMPRight)
        {
            CurrentThetaRight = NextThetaRight;
            CurrentMPRight = NextMPRight;
            NextThetaRight = CurrentThetaRight + mThetaStep;
            if(NextThetaRight > M_PI)
                NextThetaRight -= 2*M_PI;
        }
        else
        {
            break;
        }
    }
    //cout << "right: " << CurrentThetaRight << " " << CurrentMPRight << " " << i << endl;

    //Search left
    float CurrentThetaLeft = 0;
    int CurrentMPLeft = GetMapPointsInView(CurrentThetaLeft, sMapPointProject);
    float NextThetaLeft = CurrentThetaLeft - mThetaStep;
    if(NextThetaRight < -M_PI)
        NextThetaRight += 2*M_PI;
    int NextMPLeft;
    i = 0;
    while(1)
    {
        i++;
        NextMPLeft = GetMapPointsInView(NextThetaLeft, sMapPointProject);
        if(NextMPLeft >= CurrentMPLeft)
        {
            CurrentThetaLeft = NextThetaLeft;
            CurrentMPLeft = NextMPLeft;
            NextThetaLeft = CurrentThetaLeft - mThetaStep;
            if(NextThetaRight < -M_PI)
                NextThetaRight += 2*M_PI;
        }
        else
        {
            break;
        }
    }
    //cout << "left: " << CurrentThetaLeft << " " << CurrentMPLeft << " " << i << endl;

    if(CurrentMPLeft > CurrentMPRight + 10)
        mTheta = CurrentThetaLeft;
    else
        mTheta = CurrentThetaRight;

    // if(mpYunTai)
    //     mpYunTai->UpdateYunTaiPose(mTheta);

    //cout << "theta: " << mTheta / M_PI *180 << endl;

    cv::Mat Tyc = cv::Mat::zeros(4,4,CV_32F);
    Tyc.at<float>(0,0) = cos(mTheta);    Tyc.at<float>(0,2) = -sin(mTheta);
    Tyc.at<float>(1,1) = 1;
    Tyc.at<float>(2,0) = sin(mTheta);   Tyc.at<float>(2,2) = cos(mTheta);
    Tyc.at<float>(3,3) = 1;
    Tyw = Tyc * Tcw;

    //cout << "YunTai time consuming: " << t_m.toc() << endl;
}

int AttentionTranslation::GetMapPointsInView(const float theta, const set< pair<float, float> > &sMapPointProject)
{
    int nMapPointsInView = 0;

    set< pair<float, float> >::iterator it;
    if( (theta + mWindowWidth/2) > M_PI )   //right bound
    {
        for(it = sMapPointProject.lower_bound(make_pair(theta - mWindowWidth/2, -4.0f)); it != sMapPointProject.end(); it++)
        {
            if(fabs(it->second) > mWindowHeight/2)
                continue;
            
            nMapPointsInView++;
        }

        for(it = sMapPointProject.begin(); it != sMapPointProject.lower_bound(make_pair(theta + mWindowWidth/2 - M_PI, -4.0f)); it++)
        {
            if(fabs(it->second) > mWindowHeight/2)
                continue;
            
            nMapPointsInView++;
        }
    }
    else if((theta - mWindowWidth/2) < -M_PI)
    {
        for(it = sMapPointProject.begin(); it != sMapPointProject.lower_bound(make_pair(theta + mWindowWidth/2, -4.0f)); it++)
        {
            if(fabs(it->second) > mWindowHeight/2)
                continue;
            
            nMapPointsInView++;
        }

        for(it = sMapPointProject.lower_bound(make_pair(theta - mWindowWidth/2 + 2*M_PI, -4.0f)); it != sMapPointProject.end(); it++)
        {
            if(fabs(it->second) > mWindowHeight/2)
                continue;
            
            nMapPointsInView++;
        }
    }
    else
    {
        for(it = sMapPointProject.lower_bound(make_pair(theta - mWindowWidth/2, -4.0f)); it != sMapPointProject.lower_bound(make_pair(theta + mWindowWidth/2, -4.0f)); it++)
        {
            if(fabs(it->second) > mWindowHeight/2)
                continue;
            
            nMapPointsInView++;
        }
    }

    return nMapPointsInView;
}

void AttentionTranslation::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void AttentionTranslation::SetYunTai(YunTai *pYunTai)
{
    mpYunTai=pYunTai; 
}

void AttentionTranslation::GetCameraPose()
{
    mLastFrameNum = mFrameNum;
    unique_lock<mutex> lock(mMutexTcw);
    mTcw = mpTracker->mCurrentFrame.mTcw.clone();
    mFrameNum = mpTracker->mFrameNum;
}

bool AttentionTranslation::GetStatus()
{
    unique_lock<mutex> lock(mMutexbOK);
    //bool bOK = mpTracker->mbOK;
    bool bOK = mpTracker->mbATSwitch;
    mpTracker->mbATSwitch = 0;
    return bOK;
}

}