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
double K[52];
double D[52];
double DATA[52];
std::map<int,int> mymap;
std::map<int,int>::iterator it;

int8_t controller;

int id[12];

void signal_callback_handler(int signum)
{
    printf("Caught signal %d\n", signum);
    sleep(0.1);
    exit(1);
}

void controllerCallback(std_msgs::Int8 msg){
    controller = msg.data;
}

void KDInit(){
    D[51]= + 374.991817 ;
    D[30]= - 253.545633 ;
    D[27]= - 270.565562 ;
    D[42]= - 31.614125 ;
    D[49]= - 105.326736 ;
    D[44]= + 218.221682 ;
    D[39]= + 517.990897 ;
    D[37]= + 312.818379 ;
    D[50]= - 608.027630  ;
    D[29]= + 547.595842 ;
    D[6]= + 308.427742 ;
    D[41]= - 142.557234 ;


    K[51]= 4941923.895689 ;
    K[30]= 4957594.256778 ;
    K[27]= 3735270.923792 ;
    K[42]= 5080847.323548 ;
    K[49]= 5072959.998486 ;
    K[44]= 5187970.724104 ;
    K[39]= 5088851.069240 ;
    K[37]= 5033680.723307 ;
    K[50]= 5004717.819474 ;
    K[29]= 5421622.838842 ;
    K[6]= 5088146.399029 ;
    K[41]= 5032929.122178 ;

}

void mapInit(){
    mymap[513957+0]=6;
    mymap[513957+1]=44;
    mymap[513957+2]=49;

    mymap[513125+0]=42;
    mymap[513125+1]=29;
    mymap[513125+2]=41;

    mymap[514762+0]=27;
    mymap[514762+1]=37;
    mymap[514762+2]=51;

    mymap[410183+0]=39;
    mymap[410183+1]=30;
    mymap[410183+3]=50;
}

struct loadcell{
    double voltageRatio;
    double F;
};
loadcell lc[52];

static void CCONV onVoltageRatioChange(PhidgetVoltageRatioInputHandle ch, void * ctx, double voltageRatio) {
	int channel;
    int32_t SN;
	int value;
	int id;
    Phidget_getDeviceSerialNumber((PhidgetHandle)ch, &SN);
	Phidget_getChannel((PhidgetHandle)ch, &channel);
	
	// printf("VoltageRatio [%d %d]: %lf\n;", SN,channel, voltageRatio);

	value = channel+SN;
    // 通过map找到对应的id信息
    
    it = mymap.find(value);
    if (it != mymap.end()){
        // printf("Find, the value is %d\n",it->second);
        id = it->second;
    }
	lc[id].F = K[id]*voltageRatio+D[id];
    lc[id].voltageRatio = voltageRatio;
}

ros::Publisher oriDataPub;
ros::Publisher ForceDataPub;

ros::Subscriber controllerSub;

std_msgs::Float32MultiArray oriDataMsg,ForceDataMsg;
int main(int argc, char ** argv){

    ros::init(argc,argv,"read");
    ros::NodeHandle n;
    ros::Rate rate(50);
    KDInit();
	mapInit();
    signal(SIGINT, signal_callback_handler);
    controller = 0;

    oriDataPub = n.advertise<std_msgs::Float32MultiArray>("oriData",1);
    ForceDataPub = n.advertise<std_msgs::Float32MultiArray>("ForceData",1);

    controllerSub = n.subscribe("controller",1,controllerCallback);

    oriDataMsg.data.resize(12);
    ForceDataMsg.data.resize(12);

    // 设置loadcell
    PhidgetVoltageRatioInputHandle voltageRatioInput0;
	PhidgetVoltageRatioInputHandle voltageRatioInput1;
	PhidgetVoltageRatioInputHandle voltageRatioInput2;
    
	PhidgetVoltageRatioInput_create(&voltageRatioInput0);
	PhidgetVoltageRatioInput_create(&voltageRatioInput1);
	PhidgetVoltageRatioInput_create(&voltageRatioInput2);
	

	Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput0, 513957);
	Phidget_setChannel((PhidgetHandle)voltageRatioInput0, 0);
	Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput1, 513957);
	Phidget_setChannel((PhidgetHandle)voltageRatioInput1, 1);
	Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput2, 513957);
	Phidget_setChannel((PhidgetHandle)voltageRatioInput2, 2);


	PhidgetVoltageRatioInput_setOnVoltageRatioChangeHandler(voltageRatioInput0, onVoltageRatioChange, NULL);
	PhidgetVoltageRatioInput_setOnVoltageRatioChangeHandler(voltageRatioInput1, onVoltageRatioChange, NULL);
	PhidgetVoltageRatioInput_setOnVoltageRatioChangeHandler(voltageRatioInput2, onVoltageRatioChange, NULL);

	Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput0, 500);
	Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput1, 500);
	Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput2, 500);


    PhidgetVoltageRatioInput_setDataInterval(voltageRatioInput0, 50);
    PhidgetVoltageRatioInput_setDataInterval(voltageRatioInput1, 50);
    PhidgetVoltageRatioInput_setDataInterval(voltageRatioInput2, 50);


    PhidgetVoltageRatioInputHandle voltageRatioInput3;
	PhidgetVoltageRatioInputHandle voltageRatioInput4;
	PhidgetVoltageRatioInputHandle voltageRatioInput5;
    
	PhidgetVoltageRatioInput_create(&voltageRatioInput3);
	PhidgetVoltageRatioInput_create(&voltageRatioInput4);
	PhidgetVoltageRatioInput_create(&voltageRatioInput5);
	

	Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput3, 513125);
	Phidget_setChannel((PhidgetHandle)voltageRatioInput3, 0);
	Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput4, 513125);
	Phidget_setChannel((PhidgetHandle)voltageRatioInput4, 1);
	Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput5, 513125);
	Phidget_setChannel((PhidgetHandle)voltageRatioInput5, 2);


	PhidgetVoltageRatioInput_setOnVoltageRatioChangeHandler(voltageRatioInput3, onVoltageRatioChange, NULL);
	PhidgetVoltageRatioInput_setOnVoltageRatioChangeHandler(voltageRatioInput4, onVoltageRatioChange, NULL);
	PhidgetVoltageRatioInput_setOnVoltageRatioChangeHandler(voltageRatioInput5, onVoltageRatioChange, NULL);

	Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput3, 500);
	Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput4, 500);
	Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput5, 500);


    PhidgetVoltageRatioInput_setDataInterval(voltageRatioInput3, 50);
    PhidgetVoltageRatioInput_setDataInterval(voltageRatioInput4, 50);
    PhidgetVoltageRatioInput_setDataInterval(voltageRatioInput5, 50);


    PhidgetVoltageRatioInputHandle voltageRatioInput6;
	PhidgetVoltageRatioInputHandle voltageRatioInput7;
	PhidgetVoltageRatioInputHandle voltageRatioInput8;
    
	PhidgetVoltageRatioInput_create(&voltageRatioInput6);
	PhidgetVoltageRatioInput_create(&voltageRatioInput7);
	PhidgetVoltageRatioInput_create(&voltageRatioInput8);
	

	Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput6, 410183);
	Phidget_setChannel((PhidgetHandle)voltageRatioInput6, 0);
	Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput7, 410183);
	Phidget_setChannel((PhidgetHandle)voltageRatioInput7, 1);
	Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput8, 410183);
	Phidget_setChannel((PhidgetHandle)voltageRatioInput8, 3);


	PhidgetVoltageRatioInput_setOnVoltageRatioChangeHandler(voltageRatioInput6, onVoltageRatioChange, NULL);
	PhidgetVoltageRatioInput_setOnVoltageRatioChangeHandler(voltageRatioInput7, onVoltageRatioChange, NULL);
	PhidgetVoltageRatioInput_setOnVoltageRatioChangeHandler(voltageRatioInput8, onVoltageRatioChange, NULL);

	Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput6, 500);
	Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput7, 500);
	Phidget_openWaitForAttachment((PhidgetHandle)voltageRatioInput8, 500);


    PhidgetVoltageRatioInput_setDataInterval(voltageRatioInput6, 50);
    PhidgetVoltageRatioInput_setDataInterval(voltageRatioInput7, 50);
    PhidgetVoltageRatioInput_setDataInterval(voltageRatioInput8, 50);


    PhidgetVoltageRatioInputHandle voltageRatioInput9;
	PhidgetVoltageRatioInputHandle voltageRatioInput10;
	PhidgetVoltageRatioInputHandle voltageRatioInput11;
    
	PhidgetVoltageRatioInput_create(&voltageRatioInput9);
	PhidgetVoltageRatioInput_create(&voltageRatioInput10);
	PhidgetVoltageRatioInput_create(&voltageRatioInput11);
	

	Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput9, 514762);
	Phidget_setChannel((PhidgetHandle)voltageRatioInput9, 0);
	Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput10, 514762);
	Phidget_setChannel((PhidgetHandle)voltageRatioInput10, 1);
	Phidget_setDeviceSerialNumber((PhidgetHandle)voltageRatioInput11, 514762);
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
	string filename = "/home/roma/pro/Rover/src/loadcell/Data/" + to_string(t) + "OriData.csv";
   	string filename2 = "/home/roma/pro/Rover/src/loadcell/Data/" + to_string(t) + "ForceData.csv";
	outfile.open(filename,ios::app);
    outfile2.open(filename2,ios::app);
	
	outfile << "time"<< ",";
	outfile2 << "time"<< ",";
    outfile << 27 << "," <<37<<","<<51<<","<<42<<","<<29<<","<<41<<","<<6<<","<<44<<","<<49<<","<<39<<","<<30<<","<<50;
    outfile2 << 27 << "," <<37<<","<<51<<","<<42<<","<<29<<","<<41<<","<<6<<","<<44<<","<<49<<","<<39<<","<<30<<","<<50;
	outfile << endl;
	outfile2 << endl;
	while(1){
		if (controller == 1){
            std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(
            std::chrono::system_clock::now().time_since_epoch());
            int64_t time = double(ms.count());
            outfile << time <<",";
            outfile2 << time <<",";

            oriDataMsg.data[0] = lc[27].voltageRatio;
            oriDataMsg.data[1] = lc[37].voltageRatio;
            oriDataMsg.data[2] = lc[51].voltageRatio;
            oriDataMsg.data[3] = lc[42].voltageRatio;
            oriDataMsg.data[4] = lc[29].voltageRatio;
            oriDataMsg.data[5] = lc[41].voltageRatio;
            oriDataMsg.data[6] = lc[6].voltageRatio;
            oriDataMsg.data[7] = lc[44].voltageRatio;
            oriDataMsg.data[8] = lc[49].voltageRatio;
            oriDataMsg.data[9] = lc[39].voltageRatio;
            oriDataMsg.data[10] = lc[30].voltageRatio;
            oriDataMsg.data[11] = lc[50].voltageRatio;

            ForceDataMsg.data[0] = lc[27].F;
            ForceDataMsg.data[1] = lc[37].F;
            ForceDataMsg.data[2] = lc[51].F;
            ForceDataMsg.data[3] = lc[42].F;
            ForceDataMsg.data[4] = lc[29].F;
            ForceDataMsg.data[5] = lc[41].F;
            ForceDataMsg.data[6] = lc[6].F;
            ForceDataMsg.data[7] = lc[44].F;
            ForceDataMsg.data[8] = lc[49].F;
            ForceDataMsg.data[9] = lc[39].F;
            ForceDataMsg.data[10] = lc[30].F;
            ForceDataMsg.data[11] = lc[50].F;
            ForceDataPub.publish(ForceDataMsg);
            oriDataPub.publish(oriDataMsg);
            for (int i = 1;i<13;i++) {outfile << oriDataMsg.data[i-1] << ","; outfile2 << ForceDataMsg.data[i-1] << ",";}
            outfile << endl;
            outfile2 << endl;
        }
        ros::spinOnce();
		rate.sleep();
	}
	
    return 0;
}