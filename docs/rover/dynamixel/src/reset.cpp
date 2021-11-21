#include "ros/ros.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "std_msgs/Int8.h"

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

PortHandler * portHandler;
PacketHandler * packetHandler;

PortHandler * portHandler2;
PacketHandler * packetHandler2;

int main(int argc,char **argv){
    ros::init(argc,argv,"demo");
    ros::NodeHandle n;

    ros::Publisher controllerPub = n.advertise<std_msgs::Int8>("controller",1);

    std_msgs::Int8 controllerMsg;
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
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 254, ADD_OPERATING_MODE, 3, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, 254, ADD_OPERATING_MODE, 3, &dxl_error);

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 1, ADD_OPERATING_MODE, 1, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 2, ADD_OPERATING_MODE, 1, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, 1, ADD_OPERATING_MODE, 1, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, 2, ADD_OPERATING_MODE, 1, &dxl_error);
    // 设置左右旋转的profileVeloxity
    // dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 3, ADDR_PROFILE_VELOCITY, 0, &dxl_error);
    // dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 4, ADDR_PROFILE_VELOCITY, 0, &dxl_error);
    // dxl_comm_result = packetHandler->write4ByteTxRx(portHandler2, 3, ADDR_PROFILE_VELOCITY, 0, &dxl_error);
    // dxl_comm_result = packetHandler->write4ByteTxRx(portHandler2, 4, ADDR_PROFILE_VELOCITY, 0, &dxl_error);
    // 使能电机
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 254, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, 254, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    // 第一目标行为
    sleep(0.1);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 3, ADDR_GOAL_POSITION, 4092/2, &dxl_error);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 4, ADDR_GOAL_POSITION, 4092/2, &dxl_error);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler2, 3, ADDR_GOAL_POSITION, 4092/2, &dxl_error);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler2, 4, ADDR_GOAL_POSITION, 4092/2, &dxl_error);
    sleep(2);
    return 0;
}