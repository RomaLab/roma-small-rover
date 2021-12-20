#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int8.h"
#include <map>
#include <string>
#include <iostream>
#include <phidget22.h>
#include <stdio.h>
#include <chrono>
#include <ctime>
#include <fstream>
#include <unistd.h>
#include <signal.h>

using namespace std;  
double sens[3];

std::map<int,int> mymap;
std::map<int,int>::iterator it;

int8_t controller;
int32_t SN;
double kx, ky, kz;

ros::Publisher oriDataPub;
ros::Publisher ForceDataPub;
ros::Subscriber controllerSub;

std_msgs::Float32MultiArray oriDataMsg,ForceDataMsg;

void signal_callback_handler(int signum)
{
    printf("Caught signal %d\n", signum);
    sleep(0.1);
    exit(1);
}

void controllerCallback(std_msgs::Int8 msg){
    controller = msg.data;
}

void SensInit(){
    sens[0] = kx;
    sens[1] = ky;
    sens[2] = kz;
}

void mapInit(){
    mymap[SN+0]=0;
    mymap[SN+1]=1;
    mymap[SN+2]=2;
}

struct loadcell{
    double voltageRatio;
    double F;
};
loadcell lc[52];

static void CCONV onVoltageRatioChange(PhidgetVoltageRatioInputHandle ch, void * ctx, double voltageRatio) {
	int channel;
    int32_t SN1;
	int value;
	int id;
    Phidget_getDeviceSerialNumber((PhidgetHandle)ch, &SN1);
	Phidget_getChannel((PhidgetHandle)ch, &channel);
	
	// printf("VoltageRatio [%d %d]: %lf\n;", SN,channel, voltageRatio);

	value = channel+SN1;
    // 通过map找到对应的id信息
    
    it = mymap.find(value);
    if (it != mymap.end()){
        // printf("Find, the value is %d\n",it->second);
        id = it->second;
    }
	lc[id].F = 200*1000*voltageRatio/sens[id];
    lc[id].voltageRatio = voltageRatio;
}


int main(int argc, char ** argv){

    ros::init(argc,argv,"read2");
    ros::NodeHandle n("~");
    ros::Rate rate(50);
    signal(SIGINT, signal_callback_handler);
    controller = 0;

    n.param("SN", SN, 513957);
    n.param("kx", kx, 1.0);
    n.param("ky", ky, 1.0);
    n.param("kz", kz, 1.0);
    ROS_INFO("\nSN number:%d \nkx:%.3f\nky:%.3f\nkz:%.3f", SN,kx,ky,kz);
    
    SensInit();
	mapInit();
    oriDataPub = n.advertise<std_msgs::Float32MultiArray>("oriData",1);
    ForceDataPub = n.advertise<std_msgs::Float32MultiArray>("ForceData",1);
    controllerSub = n.subscribe("controller",1,controllerCallback);

    oriDataMsg.data.resize(3);
    ForceDataMsg.data.resize(3);

    // 设置loadcell
   
    PhidgetVoltageRatioInputHandle voltageRatioInput9;
	PhidgetVoltageRatioInputHandle voltageRatioInput10;
	PhidgetVoltageRatioInputHandle voltageRatioInput11;
    
	PhidgetVoltageRatioInput_create(&voltageRatioInput9);
	PhidgetVoltageRatioInput_create(&voltageRatioInput10);
	PhidgetVoltageRatioInput_create(&voltageRatioInput11);
	

	Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput9, SN);
	Phidget_setChannel((PhidgetHandle)voltageRatioInput9, 0);
	Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput10, SN);
	Phidget_setChannel((PhidgetHandle)voltageRatioInput10, 1);
	Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput11, SN);
	Phidget_setChannel((PhidgetHandle)voltageRatioInput11, 2);

	PhidgetVoltageRatioInput_setOnVoltageRatioChangeHandler(voltageRatioInput9, onVoltageRatioChange, NULL);
	PhidgetVoltageRatioInput_setOnVoltageRatioChangeHandler(voltageRatioInput10, onVoltageRatioChange, NULL);
	PhidgetVoltageRatioInput_setOnVoltageRatioChangeHandler(voltageRatioInput11, onVoltageRatioChange, NULL);

	Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput9, 500);
	Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput10, 500);
	Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput11, 500);


    PhidgetVoltageRatioInput_setDataInterval(voltageRatioInput9, 50);
    PhidgetVoltageRatioInput_setDataInterval(voltageRatioInput10, 50);
    PhidgetVoltageRatioInput_setDataInterval(voltageRatioInput11, 50);

    
    // TODO 循环体，每50m获取一次数据，并存储为csv文件

	time_t t = time(0); 
	ofstream outfile;
   	ofstream outfile2;
	string filename = "/home/roma/roma-small-rover/src/loadcell/data/" + to_string(t) + "OriData.csv";
   	string filename2 = "/home/roma/roma-small-rover/src/loadcell/data/" + to_string(t) + "ForceData.csv";
	outfile.open(filename,ios::app);
    outfile2.open(filename2,ios::app);
	
	outfile << "time"<< ",";
	outfile2 << "time"<< ",";
    outfile << 1 << "," <<2<<","<<3;
    outfile2 << 1 << "," <<2<<","<<3;
	outfile << endl;
	outfile2 << endl;
	while(1){
		if (controller == 1){
            std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(
            std::chrono::system_clock::now().time_since_epoch());
            int64_t time = double(ms.count());
            outfile << time <<",";
            outfile2 << time <<",";
            for (int id2 = 0;id2<3;id2++){
                oriDataMsg.data[id2] = lc[id2].voltageRatio;
                ForceDataMsg.data[id2] = lc[id2].F;
                // printf("%f\n", ForceDataMsg.data[id2]);
            }
            
            ForceDataPub.publish(ForceDataMsg);
            oriDataPub.publish(oriDataMsg);
            for (int i = 0;i<3;i++) {outfile << oriDataMsg.data[i] << ","; outfile2 << ForceDataMsg.data[i] << ",";}
            outfile << endl;
            outfile2 << endl;
        }
        ros::spinOnce();
		rate.sleep();
	}
	
    return 0;
}