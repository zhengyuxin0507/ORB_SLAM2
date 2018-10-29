#include "YunTai.h"

namespace ORB_SLAM2
{

YunTai::YunTai():
mTheta(0), mThetaStep(0.05), mWindowWidth(0.84), mWindowHeight(0.61),
mbFinishRequested(false), mbFinished(false)
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

void YunTai::TurnLeft(const float v)
{
    int num[]={0xFF,0x01,0x00,0x04,0x00,0x00,0x00};
    char buff[7];

    int vel = static_cast<int>(0x3F * v);
    if(vel > 0x3F)
        vel = 0x3F;
    if(vel < 0)
        vel = 0x00;

    num[4] = vel;
    num[6] = (num[1] + num[2] + num[3] + num[4] + num[5]) % 0x100;

    for(int i = 0; i < 7; i++)
        buff[i] = static_cast<char>(num[i]);

    write(fd,buff,7);
}

void YunTai::TurnRight(const float v)
{
    int num[]={0xFF,0x01,0x00,0x02,0x00,0x00,0x00};
    char buff[7];

    int vel = static_cast<int>(0x3F * v);
    if(vel > 0x3F)
        vel = 0x3F;
    if(vel < 0)
        vel = 0x00;

    num[4] = vel;
    num[6] = (num[1] + num[2] + num[3] + num[4] + num[5]) % 0x100;

    for(int i = 0; i < 7; i++)
        buff[i] = static_cast<char>(num[i]);

    write(fd,buff,7);
}

void YunTai::Stop()
{
    int num[]={0xFF,0x01,0x00,0x00,0x00,0x00,0x01};
    char buff[7];

    for(int i = 0; i < 7; i++)
        buff[i] = static_cast<char>(num[i]);

    write(fd,buff,7);
}

void YunTai::GoZero()
{
    int num[]={0xFF,0x01,0x00,0x4B,0x00,0x00,0x4C};
    char buff[7];

    for(int i = 0; i < 7; i++)
        buff[i] = static_cast<char>(num[i]);

    write(fd,buff,7);
}

void YunTai::GetYaw(int &yaw)
{
    int num[]={0xFF,0x01,0x00,0x51,0x00,0x00,0x52};
    char buff[7];
    //char recv[7];
    char* recv;
    for(int i = 0; i < 7; i++)
        buff[i] = static_cast<char>(num[i]);

    write(fd, buff, 7);
    std::cout << "get data2" << std::endl;
    int state;
    write(fd, buff, 7);
    state = read(fd, recv, 1);
    if(state > -1)
        printf("recv buff is: %s\n", recv);
    std::cout << "state: " << state << std::endl;
    //std::cout << "recv buff: ";
    //for(int i = 0; i < 7; i++)
    //    std::cout << *(recv+i) << " ";
    //std::cout << std::endl;
}

void YunTai::Run()
{
    char *dev  = "/dev/ttyUSB0";
    fd = OpenDev(dev);
    set_speed(fd,9600);

    if (set_Parity(fd,8,1,'N') == -1)  
    {
        printf("Set Parity Error/n");
        //return;
        exit(0);
    }

    GoZero();
    sleep(2);

    char command;
    char vel;
    int yaw;

    float theta;
    while(1)
    {
        {
            unique_lock<mutex> lock(mMutex);
            theta = mTheta;
        }

        if(theta > 0.09)
        {
            TurnRight(0.1);
        }
        else if(theta < -0.09)
        {
            TurnLeft(0.1);
        }
        else
        {
            Stop();
        }

        if(CheckFinish())
            break;

        usleep(50000);
    }

    Stop();
    usleep(50000);
    close(fd);

    SetFinish();
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
}

bool YunTai::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

}