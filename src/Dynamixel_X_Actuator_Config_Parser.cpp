#include <pluginlib/class_list_macros.h>

#include "Dynamixel_X_Actuator_Config_Parser.h"
#include "Dynamixel_X_Actuator_Properties.h"
#include "Dynamixel_X_Controller.h"

#include <memory>
#include <stdexcept>
#include <algorithm>
#include <cmath>

using namespace XmlRpc;
using namespace Dynamixel_X;

void Dynamixel_X_Actuator_Config_Parser::parseConfig(XmlRpc::XmlRpcValue& config, Actuator_Controller_Map controller_map) 
{
    if (config.getType() != XmlRpcValue::Type::TypeStruct)
    {
        throw std::runtime_error("actuator config parsing failed!");
    }
    
    for(auto it = config.begin(); it != config.end(); ++it)
    {
        if (it->second.getType() != XmlRpcValue::Type::TypeStruct)
        {
            throw std::runtime_error("actuator config parsing failed!");
        }
    
        std::string actuator_name = it->first;
        uint8_t actuator_id = (uint8_t)static_cast<int>(it->second["servo_id"]);
        std::string actuator_controller = static_cast<std::string>(it->second["controller_name"]);

        Dynamixel_X::Actuator_Properties_Ptr actuator = std::make_shared<Dynamixel_X_Actuator_Properties>
        (actuator_name, actuator_id);
        actuator->ccw_limit_deg = static_cast<double>(it->second["ccw_limit_deg"]);
        actuator->cw_limit_deg = static_cast<double>(it->second["cw_limit_deg"]);
        if (it->second.hasMember("zero_deg"))
        {
            actuator->zero_deg = static_cast<double>(it->second["zero_deg"]);
        }
        else
        {
            actuator->zero_deg = 180.0;
        }
        if (it->second.hasMember("max_effort_value"))
        {
            actuator->max_effort_value = fabs(static_cast<double>(it->second["max_effort_value"]));
        }

        auto controller_it = controller_map->find(actuator_controller);
        if (controller_it == controller_map->end())
        {
            throw std::runtime_error("actuator controller config missing: " + actuator_controller);
        }
        auto controller = std::dynamic_pointer_cast<Dynamixel_X_Controller>(controller_it->second);
        if (!controller)
        {
            throw std::bad_cast();
        }
        controller->addServo(actuator);
    }
}

PLUGINLIB_EXPORT_CLASS(Dynamixel_X::Dynamixel_X_Actuator_Config_Parser, Actuator_Config_Parser)