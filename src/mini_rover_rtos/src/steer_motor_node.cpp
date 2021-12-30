#include <ros/ros.h>
#include <iostream>
#include "steer_fsmlist.hpp"
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <yaml-cpp/yaml.h>
#include "multi_motor_controller.hpp"
using namespace dynamixel;

#define ADDR_TORQUE_ENABLE    64
#define ADDR_GOAL_POSITION    116
#define ADDR_GOAL_VELOCITY    104
#define ADD_OPERATING_MODE    11
#define ADDR_PROFILE_VELOCITY 112

#define BAUDRATE              4000000
#define DEVICE_NAME           "/dev/ttyUSB0"
#define PROTOCOL_VERSION      2.0



using namespace std;


int start(int argc, char **argv)
{
    /////////////////// ros init ///////////////////////
    ros::init(argc, argv, "WheelMotorController");
    ros::NodeHandle node_handle("");

    ///////////////// dynamixel init /////////////////
    DynamixelWorkbench *dxl_wb = new DynamixelWorkbench();
    bool result = false;
    const char* log;
    
    std::string port_name = "/dev/ttyUSB0";
    uint32_t baud_rate = 4000000;
    if (argc < 2)
    {
        ROS_WARN("Default port_name: /dev/ttyUSB0. Default baud_rate: 4000000");
    }
    else
    {
        port_name = argv[1];
        baud_rate = atoi(argv[2]);
    }

    result = dxl_wb->init(port_name.c_str(), baud_rate, &log);
    if (result == false)
    {
        ROS_ERROR("%s", log);
        ROS_ERROR("Please check USB port name");
        return 0;
    }

    


    std::string yaml_file = node_handle.param<std::string>("dynamixel_info", "");
    
    // YAML::Node dynamixel;
    // dynamixel = YAML::LoadFile(yaml_file.c_str());

    
    fsm_list::start();

    SetSteerPosition setPosition;
    Brake brake;
    SteerInitialize initialize;
    TorqueOff off;
    initialize.controller = new MultiMotorController(dxl_wb,yaml_file);

    while (ros::ok())
    {
        char c;
        ROS_INFO("s=SetPosition, b=Brake, i=WheelInitialize, q=Quit");
        cin >> c;
        
        switch (c)
        {
        case 's':
            ROS_INFO("Enter your goal position for wheel motor");
            for (size_t i = 0; i < 4; i++)
            {
                ROS_INFO("Motor %d: ",int(i));
                cin >> setPosition.goalPosition[i];
            }
            send_event(setPosition);
            break;
        case 'b':
            send_event(brake);
            break;
        case 'i':
            send_event(initialize);
            break;
        case 'q':
            send_event(off);
            return 0;
            break;
        default:
            break;
        }

        ros::spinOnce();
    }
    cout << "example finished" << endl;
    return 0;
    
}


void test(int argc, char **argv){
}

int main(int argc, char **argv){
    // test(argc, argv);
    start(argc,argv);
    return 0;
}

