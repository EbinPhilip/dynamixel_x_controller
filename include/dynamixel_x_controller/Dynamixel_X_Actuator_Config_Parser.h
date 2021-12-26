#ifndef __DYNAMIXEL_ACTUATOR_CONFIG_PARSER_H__
#define __DYNAMIXEL_ACTUATOR_CONFIG_PARSER_H__

#include <configurable_control_hw/Actuator_Config_Parser.h>

namespace Dynamixel_X
{
class Dynamixel_X_Actuator_Config_Parser : public Actuator_Config_Parser
{
public:
    virtual void parseConfig(XmlRpc::XmlRpcValue& config, Actuator_Controller_Map controller_map) override;
};
}

#endif