
#ifndef MULTI_MOTOR_CONTROLLER_HPP_INCLUDED
#define MULTI_MOTOR_CONTROLLER_HPP_INCLUDED
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1

typedef struct
{
  std::string item_name;
  int32_t value;
} ItemValue;

enum ControllerType{Velocity, Position};

class MultiMotorController
{
private:
    DynamixelWorkbench *dxl_wb_;
    std::string yaml_file_;
    ControllerType type_;
    std::map<std::string, uint32_t> dynamixel_;
    std::map<std::string, bool> wheel_direction_;
    std::map<std::string, int32_t> max_position_limit_;
    std::map<std::string, int32_t> min_position_limit_;
    std::vector<std::pair<std::string, ItemValue>> dynamixel_info_;
    std::map<std::string, const ControlItem*> control_items_;

    bool  getDynamixelsInfo(const std::string yaml_file);
    bool  loadDynamixels();
    bool  initDynamixels();
    bool  initControlItems();
    bool  initSDKHandlers();

public:
    MultiMotorController(DynamixelWorkbench *dxl_wb, const std::string yaml_file, ControllerType type);
    void  setGoalSpeed(const int32_t goalSpeed[4]);
    void  setGoalPosition(const int32_t goalPosition[4]);
    void  setSpeedZero();
    bool  torqueOn();
    bool  torqueOff();
    bool  initialize();
    bool  liftMotorInitialize();
};

#endif