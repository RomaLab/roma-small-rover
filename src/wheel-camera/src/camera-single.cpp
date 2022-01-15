#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <signal.h>

using namespace cv;
using namespace std;

ros::Subscriber controllerSub;
int controller;
int cameraID;
double frameWidth, frameHeight, frameFPS;
std_msgs::String fileName;

void signal_callback_handler(int signum)
{
    printf("Caught signal %d\n", signum);
    sleep(0.1);
    exit(0);
}

void controllerCallback(std_msgs::Int8 msg){
    controller = msg.data;
}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"video");
    ros::NodeHandle n("~");
    signal(SIGINT, signal_callback_handler);
    
    controller = 0;
    n.param("cameraID", cameraID, 1);
    n.param("frameWidth", frameWidth, 320.0);
    n.param("frameHeight", frameHeight, 240.0);
    n.param("frameFPS", frameFPS, 30.0);
    n.param<string>("fileName", fileName.data, "default");
    ROS_INFO("\ncameraID:%d \nframeWidth:%.1f\nframeHeight:%.1f\nframeFPS:%.1f\nfileName:%s", cameraID,frameWidth,frameHeight,frameFPS,fileName.data.c_str());

    controllerSub = n.subscribe("/switch",10,controllerCallback);
    // controllerSub = n.subscribe("/controller",1,controllerCallback);
 
    VideoCapture camera(cameraID);
    Mat3b frame;
    VideoWriter writer;

    camera.set(CAP_PROP_FRAME_WIDTH,frameWidth);
    camera.set(CAP_PROP_FRAME_HEIGHT,frameHeight);

    if(!camera.isOpened()) 
        return 1;
    camera >> frame; //取出一帧

    bool isColor = (frame.type() == CV_8UC3);
    double fps = frameFPS;      
    int codec = VideoWriter::fourcc('M', 'J', 'P', 'G');  // 选择视频的格式   
    time_t t = time(0); 

    // string filename = "/home/roma/roma-small-rover/src/wheel-camera/data/"+ fileName.data + "_"+to_string(cameraID)+".avi";     
    string filename = fileName.data + "_F" +to_string(cameraID/2+1)+".avi";
    
    writer.open(filename, codec, fps, frame.size(), isColor);

    if(controller == 0){
        ROS_INFO("Camera%d has been up, no record now.",cameraID);
    }

    while(true) {
        //grab and retrieve each frames of the video sequentially
        camera >> frame;

        imshow("Video"+to_string(cameraID), frame);
        if (controller == 1){
            writer.write(frame); 
            ROS_INFO("Camera%d has been up, recording now.",cameraID);
        }

        int c = waitKey(20);
        //exit the loop if user press "Esc" key  (ASCII value of "Esc" is 27)
        if(27 == char(c)) break;
        ros::spinOnce();
    }
    return 0;
}