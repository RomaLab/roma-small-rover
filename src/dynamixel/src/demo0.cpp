#include "ros/ros.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "std_msgs/Int8.h"
#include <signal.h>


using namespace dynamixel;

#define ADDR_TORQUE_ENABLE    64
#define ADDR_GOAL_POSITION    116
#define ADDR_GOAL_VELOCITY    104
#define ADD_OPERATING_MODE    11
#define ADDR_PROFILE_VELOCITY 112

#define BAUDRATE              57600
#define DEVICE_NAME           "/dev/ttyUSB0"
#define DEVICE_NAME2          "/dev/ttyUSB1"
#define PROTOCOL_VERSION      2.0

#define angle 0
#define targetVelocity 20
#define profileVelocity 10
// 总共运行11秒
int targetPosition = 4092/2-int((angle)/360.0*4092);
ros::Publisher controllerPub;

PortHandler * portHandler;
PacketHandler * packetHandler;

PortHandler * portHandler2;
PacketHandler * packetHandler2;
std_msgs::Int8 controllerMsg;

void signal_callback_handler(int signum)
{
    printf("Caught signal %d\n", signum);
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;
    controllerMsg.data = 0;
    controllerPub.publish(controllerMsg);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 1, ADDR_GOAL_VELOCITY, 0, &dxl_error);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 2, ADDR_GOAL_VELOCITY, 0, &dxl_error);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler2, 1, ADDR_GOAL_VELOCITY, 0, &dxl_error);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler2, 2, ADDR_GOAL_VELOCITY, 0, &dxl_error);
    sleep(0.1);
    exit(0);
}




int main(int argc,char **argv){
    ros::init(argc,argv,"demo");
    ros::NodeHandle n;
    signal(SIGINT, signal_callback_handler);
    controllerPub = n.advertise<std_msgs::Int8>("controller",1);

    
    controllerMsg.data = 1;

    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    portHandler = PortHandler::getPortHandler(DEVICE_NAME);
    packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    portHandler2 = PortHandler::getPortHandler(DEVICE_NAME2);
    packetHandler2 = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!portHandler->openPort()) {
        ROS_ERROR("Failed to open the port!");
        return -1;
    }

    if (!portHandler->setBaudRate(BAUDRATE)) {
        ROS_ERROR("Failed to set the baudrate!");
        return -1;
    }

    if (!portHandler2->openPort()) {
        ROS_ERROR("Failed to open the port!");
        return -1;
    }

    if (!portHandler2->setBaudRate(BAUDRATE)) {
        ROS_ERROR("Failed to set the baudrate!");
        return -1;
    }

    // 设置模式；
    
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 1, ADD_OPERATING_MODE, 1, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 2, ADD_OPERATING_MODE, 1, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, 1, ADD_OPERATING_MODE, 1, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, 2, ADD_OPERATING_MODE, 1, &dxl_error);
    // 设置左右旋转的profileVeloxity
    // 使能电机
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 1, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, 1, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 2, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, 2, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    // 第一目标行为
    // 第二目标行为
    sleep(1);
    controllerPub.publish(controllerMsg);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 1, ADDR_GOAL_VELOCITY, targetVelocity, &dxl_error);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 2, ADDR_GOAL_VELOCITY, -targetVelocity, &dxl_error);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler2, 1, ADDR_GOAL_VELOCITY, -targetVelocity, &dxl_error);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler2, 2, ADDR_GOAL_VELOCITY, +targetVelocity, &dxl_error);
    // 第三目标行为
    sleep(8);

    controllerMsg.data = 0;
    controllerPub.publish(controllerMsg);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 1, ADDR_GOAL_VELOCITY, 0, &dxl_error);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 2, ADDR_GOAL_VELOCITY, 0, &dxl_error);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler2, 1, ADDR_GOAL_VELOCITY, 0, &dxl_error);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler2, 2, ADDR_GOAL_VELOCITY, 0, &dxl_error);
    sleep(0.1);
    exit(0);
    
    return 0;
}