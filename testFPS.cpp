#include <opencv2/opencv.hpp>
#include <raspicam_cv.h>
#include <iostream>
#include <chrono>
#include <ctime>
#include <wiringPi.h>

using namespace std;
using namespace cv;
using namespace raspicam;
int LeftLanePos,RightLanePos,laneCenter,frameCenter,Result;

Mat frame,frameGray,Matrix,framePers,frameTrashe,frameEdge,frameFinal,ROI_Lane,frameFinalDuplicate;
vector<int> histrogramLane;
RaspiCam_Cv Camera;

stringstream ss;

Point2f Source[] = {Point2f(60,150) , Point2f(360,150),Point2f(35,200),Point2f(390,200)};
Point2f Destination[] = {Point2f(100,0) , Point2f(330,0),Point2f(100,240),Point2f(330,240)};


 void Setup ( int argc,char **argv, RaspiCam_Cv &Camera )
  {
    Camera.set ( CAP_PROP_FRAME_WIDTH,  ( "-w",argc,argv,400 ) );
    Camera.set ( CAP_PROP_FRAME_HEIGHT,  ( "-h",argc,argv,240 ) );
    Camera.set ( CAP_PROP_BRIGHTNESS, ( "-br",argc,argv,50 ) );
    Camera.set ( CAP_PROP_CONTRAST ,( "-co",argc,argv,50 ) );
    Camera.set ( CAP_PROP_SATURATION,  ( "-sa",argc,argv,50 ) );
    Camera.set ( CAP_PROP_GAIN,  ( "-g",argc,argv ,50 ) );
    Camera.set ( CAP_PROP_FPS,  ( "-fps",argc,argv,0));

}

void Percpective()
{
    line(frame,Source[0],Source[1],Scalar(0,0,255),2);
    line(frame,Source[1],Source[3],Scalar(0,0,255),2);
    line(frame,Source[3],Source[2],Scalar(0,0,255),2);
    line(frame,Source[2],Source[0],Scalar(0,0,255),2);
    
 // line(frame,Destination[0],Destination[1],Scalar(0,255,0),2);
  // line(frame,Destination[1],Destination[3],Scalar(0,255,0),2);
  //line(frame,Destination[3],Destination[2],Scalar(0,255,0),2);
  //line(frame,Destination[2],Destination[0],Scalar(0,255,0),2);
    
    
    Matrix = getPerspectiveTransform(Source, Destination);
    warpPerspective(frame,framePers,Matrix,Size(400,240));
}

void Threshold()
{
cvtColor(framePers,frameGray,COLOR_RGB2GRAY);   
inRange(frameGray,180,255,frameTrashe); 
Canny(frameTrashe,frameEdge,0,600,3,false);
add(frameTrashe,frameEdge,frameFinal);
cvtColor(frameFinal,frameFinal,COLOR_GRAY2RGB);
cvtColor(frameFinal,frameFinalDuplicate,COLOR_RGB2BGR);
}

void Histrogram()
{
    histrogramLane.resize(400);
    histrogramLane.clear();
    for(int i=0;i < 400 ; i++)
    {
        ROI_Lane = frameFinalDuplicate(Rect(i,140,1,100));
        divide(255 , ROI_Lane,ROI_Lane);
        histrogramLane.push_back((int)sum(ROI_Lane)[0]);
    }
    
}

void LaneFinder()
{
    vector<int> :: iterator LeftPtr;
    LeftPtr = max_element(histrogramLane.begin(),histrogramLane.begin()+200);
    LeftLanePos = distance(histrogramLane.begin(),LeftPtr);
    
    vector<int> :: iterator RightPtr;
    RightPtr = max_element(histrogramLane.begin()+201,histrogramLane.end());
    RightLanePos = distance(histrogramLane.begin(),RightPtr);
    
    line(frameFinal,Point2f(LeftLanePos,0),Point2f(LeftLanePos,240),Scalar(0,255,0),2);
    line(frameFinal,Point2f(RightLanePos,0),Point2f(RightLanePos,240),Scalar(0,255,0),2);
    
    cout<<"Left Pos = "<<LeftLanePos<<endl;
    cout<<"Right Pos = "<<RightLanePos<<endl;
    
}

void LaneCenter()
{
    laneCenter = (((RightLanePos - LeftLanePos)/2 )+ LeftLanePos);
    frameCenter = 211;
    line(frameFinal,Point2f(laneCenter , 0),Point2f(laneCenter , 240),Scalar(0,255,0),3);
    line(frameFinal,Point2f(frameCenter , 0),Point2f(frameCenter , 240),Scalar(255,0,0),3);
    Result = laneCenter - frameCenter;
}

void Capture()
{
 Camera.grab();
    Camera.retrieve( frame);
    //cvtColor(frame , frame1 , COLOR_BGR2RGB);
}

int main(int argc,char **argv)
{
	wiringPiSetup();
    pinMode(21 , OUTPUT);
    pinMode(22 , OUTPUT);
    pinMode(23 , OUTPUT);
    pinMode(24 , OUTPUT);
    
        
	Setup(argc, argv, Camera);
	cout<<"Connecting to camera"<<endl;
	if (!Camera.open())
	{
		
	cout<<"Failed to Connect"<<endl;
     }
     
     cout<<"Camera Id = "<<Camera.getId()<<endl;
     
     
     
    
    while(1)
    {
    auto start = std::chrono::system_clock::now();
    Capture();
    Percpective();
    Threshold();
    Histrogram();
   LaneFinder();
   LaneCenter();
    if(Result == 0)
    {
        digitalWrite(21 , 0);//decimal 0
        digitalWrite(22 , 0);
        digitalWrite(23 , 0);
        digitalWrite(24 , 0);
        cout<<"FORWARD"<<endl;
    }
    
    else if(Result > 0 && Result < 10)
    {
        digitalWrite(21 , 1);//decimal 1
        digitalWrite(22 , 0);
        digitalWrite(23 , 0);
        digitalWrite(24 , 0);
        cout<<"Right 1"<<endl;
    }
    else if(Result >= 10 && Result < 20)
    {
        digitalWrite(21 , 0);//decimal 2
        digitalWrite(22 , 1);
        digitalWrite(23 , 0);
        digitalWrite(24 , 0);
        cout<<"Right 2"<<endl;
    }
    else if(Result >= 20)
    {
        digitalWrite(21 , 1);//decimal 3
        digitalWrite(22 , 1);
        digitalWrite(23 , 0);
        digitalWrite(24 , 0);
        //cout<<"Right 3"<<endl;
    }
    
    else if(Result < 0 && Result > -10)
    {
        digitalWrite(21 , 0);//decimal 4
        digitalWrite(22 , 0);
        digitalWrite(23 , 1);
        digitalWrite(24 , 0);
        cout<<"Left 1"<<endl;
    }
    else if(Result <= -10 && Result > -20)
    {
        digitalWrite(21 , 1);//decimal 5
        digitalWrite(22 , 0);
        digitalWrite(23 , 1);
        digitalWrite(24 , 0);
        cout<<"Left 2"<<endl;
    }
    else if(Result <= -20)
    {
        digitalWrite(21 , 0);//decimal 6
        digitalWrite(22 , 1);
        digitalWrite(23 , 1);
        digitalWrite(24 , 0);
        cout<<"Left 3"<<endl;
    }
    ss.str (" ");
    ss.clear();
    ss<<"Result: "<<Result;
    putText(frame , ss.str(),Point2f(1,50),0,1,Scalar(0,0,255),2);
    
    namedWindow("orignal" , WINDOW_KEEPRATIO);
    moveWindow("orignal" , 0,100);
    resizeWindow("orignal" , 640,480);
    imshow("orignal" , frame);
    
    namedWindow("Perspective" , WINDOW_KEEPRATIO);
    moveWindow("Perspective" , 640,100);
    resizeWindow("Perspective" , 640,480);
    imshow("Perspective" , framePers);
    
    namedWindow("Final" , WINDOW_KEEPRATIO);
    moveWindow("Final" , 1280,100);
    resizeWindow("Final" , 640,480);
    imshow("Final" , frameFinal);
    
    auto end = std::chrono::system_clock::now();
  

    std::chrono::duration<double> elapsed_seconds = end-start;
    
    float t = elapsed_seconds.count();
    int FPS = 1/t;
    //cout<<"FPS = "<<FPS<<endl;
    
    waitKey(1);
    
    
    }

    
    return 0;
     
}
