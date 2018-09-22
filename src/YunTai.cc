#include "YunTai.h"

namespace ORB_SLAM2
{

YunTai::YunTai():
mTheta(0), mThetaStep(0.05), mWindowWidth(0.84), mWindowHeight(0.61),
mbStopRequested(false), mbStopped(false), mbFinishRequested(false), mbFinished(false)
{
    //TODO:
}

/*YunTai::~YunTai()
{
    //TODO:
}*/

void YunTai::GetYunTaiPose(const cv::Mat &Tcw, cv::Mat &Tyw)
{
    TicToc t_m;

    const cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    const cv::Mat tcw = Tcw.rowRange(0,3).col(3);

    mvMapPointProject.clear();

    //vector<MapPoint*> vMapPoints = mpMap->GetAllMapPoints();
    vector<MapPoint*> vMapPoints;
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

            //float theta = atan2(zc, xc);
            float theta = atan2(xc, zc);
            float gama = atan(-yc/sqrt(zc*zc+xc*xc));
            
            mvMapPointProject.push_back(make_pair(theta, gama));
        }
    }

    set< pair<float, float> > sMapPointProject;
    for(size_t i = 0; i < mvMapPointProject.size(); i++)
    {
        sMapPointProject.insert(make_pair(mvMapPointProject[i].first, mvMapPointProject[i].second));
    }

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
            cout << "NextMPRight: " << NextMPRight << endl;
        }
        else
        {
            break;
        }
    }
    cout << "right: " << CurrentThetaRight << " " << CurrentMPRight << " " << i << endl;

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
    cout << "left: " << CurrentThetaLeft << " " << CurrentMPLeft << " " << i << endl;

    if(CurrentMPLeft > CurrentMPRight)
        mTheta = CurrentThetaLeft;
    else
        mTheta = CurrentThetaRight;

    cout << "theta: " << mTheta / M_PI *180 << endl;

    cv::Mat Tyc = cv::Mat::zeros(4,4,CV_32F);
    Tyc.at<float>(0,0) = cos(mTheta);    Tyc.at<float>(0,2) = -sin(mTheta);
    Tyc.at<float>(1,1) = 1;
    Tyc.at<float>(2,0) = sin(mTheta);   Tyc.at<float>(2,2) = cos(mTheta);
    Tyc.at<float>(3,3) = 1;
    Tyw = Tyc * Tcw;

    cout << "YunTai time consuming: " << t_m.toc() << endl;
}

int YunTai::GetMapPointsInView(const float theta, const set< pair<float, float> > &sMapPointProject)
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

void YunTai::UpdateYunTaiPose(const float theta)
{
    unique_lock<mutex> lock(mMutex);

    mTheta = theta; 
}

void YunTai::Run()
{
    float theta;
    while(1)
    {
        {
            unique_lock<mutex> lock(mMutex);
            theta = mTheta;
        }

        cout << "zyx flag: " << theta << endl;

        /*if(Stop())
        {
            // Safe area to stop
            while(isStopped() && !CheckFinish())
            {
                usleep(3000);
            }

            if(CheckFinish())
                break;
        }*/

        if(CheckFinish())
            break;

        usleep(100000);
    }

    SetFinish();
}

bool YunTai::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested)
    {
        mbStopped = true;
        cout << "YunTai Control STOP" << endl;
        return true;
    }

    return false;
}

bool YunTai::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool YunTai::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

bool YunTai::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void YunTai::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

void YunTai::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
}

bool YunTai::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

}