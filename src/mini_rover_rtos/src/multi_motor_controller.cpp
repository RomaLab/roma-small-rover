#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include "multi_motor_controller.hpp"

using namespace std;

MultiMotorController::MultiMotorController(DynamixelWorkbench *dxl_wb, const std::string yaml_file, ControllerType type)
{
    dxl_wb_ = dxl_wb;
    yaml_file_ = yaml_file;
    type_ = type;
    // ROS_INFO("MultiMotorController");
}

void MultiMotorController::setGoalSpeed(const int32_t goalSpeed[4])
{
    uint8_t id_array[dynamixel_.size()];
    int32_t dynamixel_velocity[dynamixel_.size()];
    uint8_t id_cnt = 0;
    const char *log = NULL;
    for (auto const &dxl : dynamixel_)
    {
        ROS_INFO("Motor %d Velocity: %d", int(dxl.second), int(goalSpeed[id_cnt]));
        id_array[id_cnt] = (uint8_t)dxl.second;
        if (wheel_direction_[dxl.first])
        {
            dynamixel_velocity[id_cnt] = int32_t(goalSpeed[id_cnt]);
        }
        else
        {
            dynamixel_velocity[id_cnt] = -int32_t(goalSpeed[id_cnt]);
        }

        id_cnt++;
    }
    bool result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY, id_array, dynamixel_.size(), dynamixel_velocity, 1, &log);
    if (result==false)
    {
        ROS_ERROR("log: %s",log);
    }
    
    
}

void MultiMotorController::setGoalPosition(const int32_t goalPosition[4])
{
    uint8_t id_array[dynamixel_.size()];
    int32_t dynamixel_position[dynamixel_.size()];
    uint8_t id_cnt = 0;
    const char *log = NULL;
    for (auto const &dxl : dynamixel_)
    {
        ROS_INFO("Motor %d Position: %d", int(dxl.second), int(goalPosition[id_cnt]));
        id_array[id_cnt] = (uint8_t)dxl.second;
        if (goalPosition[id_cnt]<max_position_limit_[dxl.first] && \
            goalPosition[id_cnt]>min_position_limit_[dxl.first])
        {
            dynamixel_position[id_cnt] = goalPosition[id_cnt];
        }
        id_cnt++;
    }
    bool result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, id_array, dynamixel_.size(), dynamixel_position, 1, &log);
}

void MultiMotorController::setSpeedZero()
{
    // Set goal velocity to zero
    const int32_t goalSpeed[4] = {0,0,0,0};
    setGoalSpeed(goalSpeed);
}

bool MultiMotorController::torqueOn()
{
    // TODO: Set motors Torque on
    cout << "Motor Torque on" << endl;
    return true;
}

bool MultiMotorController::torqueOff()
{

    ROS_INFO("Motor Torque off");
    // Set motors Torque off
    bool result = true;
    for (auto const &dxl : dynamixel_)
    {
        result = dxl_wb_->torqueOff((uint8_t)dxl.second);
        ROS_INFO("Torque off Motor %d", (int)dxl.second);
        if (result == false)
        {
            ROS_ERROR("%s", dxl.first.c_str());
        }
    }
    return result;
}

bool MultiMotorController::liftMotorInitialize(){
    bool result = false;

    result = getDynamixelsInfo(yaml_file_);
    if (result == false)
    {
        ROS_ERROR("Please check YAML file");
    }

    result = loadDynamixels();
    if (result == false)
    {
        ROS_ERROR("Please check Dynamixel ID or BaudRate");
        return false;
    }
    else
    {
        ROS_INFO("Dynamixel Load Successed");
    }

    result = initDynamixels();
    if (result == false)
    {
        ROS_ERROR("Please check control table (http://emanual.robotis.com/#control-table)");
        return 0;
    }
    const char *log;
    for (auto const &dxl : dynamixel_)
    {
        for (auto const &info : dynamixel_info_)
        {
            if (dxl.first == info.first)
            {
                if (info.second.item_name=="Goal_Position")
                {
                    bool result = dxl_wb_->itemWrite((uint8_t)dxl.second, info.second.item_name.c_str(), info.second.value, &log);
                    if (result == false)
                    {
                        ROS_ERROR("%s", log);
                        ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[Name : %s, ID : %d]", info.second.value, info.second.item_name.c_str(), dxl.first.c_str(), dxl.second);
                        return false;
                    }
                }
            }
        }
    }
    return result;
}

bool MultiMotorController::initialize()
{
    bool result = false;

    result = getDynamixelsInfo(yaml_file_);
    if (result == false)
    {
        ROS_ERROR("Please check YAML file");
    }

    result = loadDynamixels();
    if (result == false)
    {
        ROS_ERROR("Please check Dynamixel ID or BaudRate");
        return false;
    }
    else
    {
        ROS_INFO("Dynamixel Load Successed");
    }

    result = initDynamixels();
    if (result == false)
    {
        ROS_ERROR("Please check control table (http://emanual.robotis.com/#control-table)");
        return 0;
    }

    result = initControlItems();
    if (result == false)
    {
        ROS_ERROR("Please check control items");
        return 0;
    }

    result = initSDKHandlers();
    if (result == false)
    {
        ROS_ERROR("Failed to set Dynamixel SDK Handler");
        return 0;
    }
    else
    {
        ROS_INFO("Dynamixel SDK Handler Set Successed");
    }

    return result;
}

bool MultiMotorController::getDynamixelsInfo(const std::string yaml_file)
{
    YAML::Node dynamixel;
    dynamixel = YAML::LoadFile(yaml_file.c_str());

    if (dynamixel == NULL)
        return false;

    for (YAML::const_iterator it_file = dynamixel.begin(); it_file != dynamixel.end(); it_file++)
    {
        std::string name = it_file->first.as<std::string>();
        if (name.size() == 0)
        {
            continue;
        }

        YAML::Node item = dynamixel[name];
        for (YAML::const_iterator it_item = item.begin(); it_item != item.end(); it_item++)
        {
            std::string item_name = it_item->first.as<std::string>();
            int32_t value = it_item->second.as<int32_t>();

            if (item_name == "ID")
            {
                dynamixel_[name] = value;
                min_position_limit_[name]=0;
                max_position_limit_[name]=4095;
                wheel_direction_[name] = true;
            }
            else if (item_name == "Direction")
            {
                wheel_direction_[name] = bool(value);
            }
            else
            {
                if (item_name=="Min_Position_Limit"){
                    min_position_limit_[name]=value;
                }else if(item_name=="Max_Position_Limit"){
                    max_position_limit_[name]=value;
                }
                ItemValue item_value = {item_name, value};
                std::pair<std::string, ItemValue> info(name, item_value);

                dynamixel_info_.push_back(info);
            }
        }
    }
    return true;
}

bool MultiMotorController::loadDynamixels(void)
{
    bool result = false;
    const char *log;

    for (auto const &dxl : dynamixel_)
    {
        uint16_t model_number = 0;
        result = dxl_wb_->ping((uint8_t)dxl.second, &model_number, &log);
        if (result == false)
        {
            ROS_ERROR("%s", log);
            ROS_ERROR("Can't find Dynamixel ID '%d'", dxl.second);
            return result;
        }
        else
        {
            ROS_INFO("Name : %s, ID : %d, Model Number : %d", dxl.first.c_str(), dxl.second, model_number);
        }
    }
    return result;
}

bool MultiMotorController::initDynamixels(void)
{
    const char *log;

    for (auto const &dxl : dynamixel_)
    {
        dxl_wb_->torqueOff((uint8_t)dxl.second);

        for (auto const &info : dynamixel_info_)
        {
            if (dxl.first == info.first)
            {
                if (info.second.item_name != "ID" && info.second.item_name != "Baud_Rate")
                {
                    bool result = dxl_wb_->itemWrite((uint8_t)dxl.second, info.second.item_name.c_str(), info.second.value, &log);
                    if (result == false)
                    {
                        ROS_ERROR("%s", log);
                        ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[Name : %s, ID : %d]", info.second.value, info.second.item_name.c_str(), dxl.first.c_str(), dxl.second);
                        return false;
                    }
                }
            }
        }

        dxl_wb_->torqueOn((uint8_t)dxl.second);
    }

    return true;
}

bool MultiMotorController::initControlItems(void)
{
    bool result = false;
    const char *log = NULL;

    auto it = dynamixel_.begin();

    const ControlItem *goal_position = dxl_wb_->getItemInfo(it->second, "Goal_Position");
    if (goal_position == NULL)
        return false;

    const ControlItem *goal_velocity = dxl_wb_->getItemInfo(it->second, "Goal_Velocity");
    if (goal_velocity == NULL)
        goal_velocity = dxl_wb_->getItemInfo(it->second, "Moving_Speed");
    if (goal_velocity == NULL)
        return false;

    const ControlItem *present_position = dxl_wb_->getItemInfo(it->second, "Present_Position");
    if (present_position == NULL)
        return false;

    const ControlItem *present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Velocity");
    if (present_velocity == NULL)
        present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Speed");
    if (present_velocity == NULL)
        return false;

    const ControlItem *present_current = dxl_wb_->getItemInfo(it->second, "Present_Current");
    if (present_current == NULL)
        present_current = dxl_wb_->getItemInfo(it->second, "Present_Load");
    if (present_current == NULL)
        return false;

    control_items_["Goal_Position"] = goal_position;
    control_items_["Goal_Velocity"] = goal_velocity;

    control_items_["Present_Position"] = present_position;
    control_items_["Present_Velocity"] = present_velocity;

    control_items_["Present_Current"] = present_current;

    return true;
}

bool MultiMotorController::initSDKHandlers(void)
{
    bool result = false;
    const char *log = NULL;

    auto it = dynamixel_.begin();
    

    result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Position"]->address, control_items_["Goal_Position"]->data_length, &log);

    result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Velocity"]->address, control_items_["Goal_Velocity"]->data_length, &log);
    // }
    if (result == false)
    {
        ROS_ERROR("%s", log);
        return result;
    }
    else
    {
        ROS_INFO("%s", log);
    }
    

    
    if (dxl_wb_->getProtocolVersion() == 2.0f)
    {
        uint16_t start_address = std::min(control_items_["Present_Position"]->address, control_items_["Present_Current"]->address);

        /* 
      As some models have an empty space between Present_Velocity and Present Current, read_length is modified as below.
    */
        // uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length;
        uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length + 2;

        result = dxl_wb_->addSyncReadHandler(start_address,
                                             read_length,
                                             &log);
        if (result == false)
        {
            ROS_ERROR("%s", log);
            return result;
        }
    }

    return result;
}