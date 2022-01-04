#include <ros/ros.h>
#include <iostream>
#include "rover_fsmlist.hpp"
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include "multi_motor_controller.hpp"
#include <signal.h>
#include <std_msgs/Int8.h>
using namespace dynamixel;
using namespace std;

void signal_callback_handler(int signum)
{
    printf("Caught signal %d\n", signum);
    send_event(TorqueOff());
    sleep(0.1);
    exit(0);
}


int start(int argc, char **argv)
{
    /////////////////// ros init ///////////////////////
    ros::init(argc, argv, "WheelMotorController");
    ros::NodeHandle node_handle("");
    ros::Publisher switch_pub = node_handle.advertise<std_msgs::Int8>("switch", 1000);
    std_msgs::Int8 msg;
    msg.data=0;
    ros::Rate loop_rate(10);
    signal(SIGINT, signal_callback_handler);
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

    std::string wheel_yaml_file = node_handle.param<std::string>("wheel_info", "");
    std::string steer_yaml_file = node_handle.param<std::string>("steer_info", "");
    std::string lift_yaml_file  = node_handle.param<std::string>("lift_info", "");
    
    
    fsm_list::start();

    SetRoverSpeed setSpeed;
    RoverInitialize initialize;

    initialize.wheel_controller = new MultiMotorController(dxl_wb,wheel_yaml_file,Velocity);
    initialize.steer_controller = new MultiMotorController(dxl_wb,steer_yaml_file,Position);
    initialize.lift_controller = new MultiMotorController(dxl_wb,lift_yaml_file,Position);
    
    while (ros::ok())
    {
        char c;
        ROS_INFO("i=initialize, 1=Experiment1, 2=Experiment2, 3=Experiment3, b=Brake, q=Quit");
        cin >> c;
        
        switch (c)
        {
        case 'i':
            send_event(initialize);
            break;
        case '1':
            msg.data=1;
            switch_pub.publish(msg);
            send_event(ExecExperiment1());
            msg.data=0;
            switch_pub.publish(msg);
            ROS_INFO("Back to Center? Enter Y/N");
            cin >> c;
            switch (c)
            {
            case 'Y':
            case 'y':
                send_event(BackToCenter());
                break;
            default:
                send_event(RoverBrake());
                break;
            }
            break;
        case '2':
            msg.data=1;
            switch_pub.publish(msg);
            send_event(ExecExperiment2());
            usleep(500000);
            send_event(BackToCenter());
            usleep(500000);
            send_event(ExecExperiment2());
            usleep(500000);
            msg.data=0;
            switch_pub.publish(msg);
            ROS_INFO("Back to Center? Enter Y/N");
            cin >> c;
            switch (c)
            {
            case 'Y':
            case 'y':
                send_event(BackToCenter());
                break;
            default:
                send_event(RoverBrake());
                break;
            }
            break;
        case '3':
            msg.data=1;
            switch_pub.publish(msg);
            send_event(ExecExperiment3());
            msg.data=0;
            switch_pub.publish(msg);
            ROS_INFO("Back to Center? Enter Y/N");
            cin >> c;
            switch (c)
            {
            case 'Y':
            case 'y':
                send_event(BackToCenter());
                break;
            default:
                send_event(RoverBrake());
                break;
            }
            break;
        case 'q':
            send_event(TorqueOff());
            return 0;
            break;
        default:
            break;
        }

        ros::spinOnce();
        loop_rate.sleep();
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

