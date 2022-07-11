#ifndef __DYNAMIXEL_X_ACTUATOR_PROPERTIES_H__
#define __DYNAMIXEL_X_ACTUATOR_PROPERTIES_H__

#include <configurable_control_hw/Actuator_Properties.h>
#include <rotational_units/Rotational_Units.h>

#include <memory>
#include <string>

namespace Dynamixel_X
{

const std::string ACTUATOR_NAME_STR = "dynamixel_x_actuator";

struct Dynamixel_X_Actuator_Properties : public Actuator_Properties
{
    Dynamixel_X_Actuator_Properties(const std::string& name, uint8_t id_number)
        : servo_id(id_number),
          error_code(0),
          bad_response_count(0),
          max_effort_value(1000.0)
    {
        actuator_name =  name;
        actuator_type = ACTUATOR_NAME_STR;
    }

    uint8_t servo_id;
    RUnits::Degrees cw_limit_deg;
    RUnits::Degrees ccw_limit_deg;
    RUnits::Degrees zero_deg;
    unsigned int profile_acceleration;
    double max_effort_value;

    uint8_t bad_response_count;
    uint8_t error_code;
    std::string error_details;
};

typedef std::shared_ptr<Dynamixel_X_Actuator_Properties> Actuator_Properties_Ptr;

}

#endif