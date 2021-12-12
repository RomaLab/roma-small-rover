#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include <signal.h>

using namespace cv;
using namespace std;

int controller;

void signal_callback_handler(int signum)
{
    printf("Caught signal %d\n", signum);
    sleep(0.1);
    exit(0);
}

void controllerCallback(std_msgs::Int8 msg){
    controller = msg.data;
}
ros::Subscriber controllerSub;

int main(int argc, char *argv[])
{
    controller = 0;
    ros::init(argc,argv,"video");
    ros::NodeHandle n;
    signal(SIGINT, signal_callback_handler);
    
    controllerSub = n.subscribe("/controller",1,controllerCallback);

    VideoCapture camera0(2);
    camera0.set(CAP_PROP_FRAME_WIDTH,320);
    camera0.set(CAP_PROP_FRAME_HEIGHT,240);
 
    VideoCapture camera1(0);
    camera1.set(CAP_PROP_FRAME_WIDTH,320);
    camera1.set(CAP_PROP_FRAME_HEIGHT,240);

    VideoCapture camera2(4);
    camera2.set(CAP_PROP_FRAME_WIDTH,320);
    camera2.set(CAP_PROP_FRAME_HEIGHT,240);
 
    VideoCapture camera3(6);
    camera3.set(CAP_PROP_FRAME_WIDTH,320);
    camera3.set(CAP_PROP_FRAME_HEIGHT,240);

    if( !camera0.isOpened() ) return 1;
    if( !camera1.isOpened() ) return 1;
    if( !camera2.isOpened() ) return 1;
    if( !camera3.isOpened() ) return 1;
    
    Mat3b frame0,frame1,frame2,frame3;

    camera0>>frame0; //取出一帧
    camera1 >> frame1;
    camera2 >> frame2;
    camera3 >> frame3;
    VideoWriter writer1,writer2,writer3,writer4;

    bool isColor = (frame0.type() == CV_8UC3);
    double fps = 30.0;      
    int codec = VideoWriter::fourcc('M', 'J', 'P', 'G');  // 选择视频的格式   
    time_t t = time(0); 

    string filename1 = "/home/roma/pro/Rover/src/loadcell/Video/"+to_string(t) + "data"+to_string(1)+".avi"; 
    string filename2 = "/home/roma/pro/Rover/src/loadcell/Video/"+to_string(t) + "data"+to_string(2)+".avi"; 
    string filename3 = "/home/roma/pro/Rover/src/loadcell/Video/"+to_string(t) + "data"+to_string(3)+".avi"; 
    string filename4 = "/home/roma/pro/Rover/src/loadcell/Video/"+to_string(t) + "data"+to_string(4)+".avi"; 
    
    
    writer1.open(filename1, codec, fps, frame0.size(), isColor);
    writer2.open(filename2, codec, fps, frame1.size(), isColor);
    writer3.open(filename3, codec, fps, frame2.size(), isColor);
    writer4.open(filename4, codec, fps, frame3.size(), isColor);


    while(true) {
 
        //grab and retrieve each frames of the video sequentially
        
        camera0 >> frame0;
        camera1 >> frame1;
        camera2 >> frame2;
        camera3 >> frame3;

        imshow("Video0", frame0);
        imshow("Video2", frame1);
        imshow("Video4", frame2);
        imshow("Video6", frame3);
        if (controller == 1){
            writer1.write(frame0); 
            writer2.write(frame1); 
            writer3.write(frame2); 
            writer4.write(frame3); 
        }
        

        //std::cout << frame1.rows() << std::endl;
        //wait for 40 milliseconds
 
        int c = waitKey(20);
        //exit the loop if user press "Esc" key  (ASCII value of "Esc" is 27)
        if(27 == char(c)) break;
        ros::spinOnce();
 
    }
    return 0;
}