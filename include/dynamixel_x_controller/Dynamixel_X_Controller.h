#ifndef __DYNAMIXEL_X_CONTROLLER_H__
#define __DYNAMIXEL_X_CONTROLLER_H__

#include <configurable_control_hw/Actuator_Controller.h>

#include "Dynamixel_X_Actuator_Properties.h"
#include <dynamixel_sdk/dynamixel_sdk.h>

#include <rotational_units/Rotational_Units.h>

#include <map>
#include <string>
#include <memory>

namespace Dynamixel_X
{

const int MAX_POSITION_LIMIT_ADDRESS = 48;
const int MIN_POSITION_LIMIT_ADDRESS = 52;
const int HOMING_OFFSET_ADDRESS = 20;
const int TORQUE_ENABLE_ADDRESS = 64;
const int LED_ADDRESS = 65;
const int HARDWARE_ERROR_STATUS = 70;
const int PROFILE_ACCELERATION = 108;
const int PRESENT_LOAD_ADDRESS = 126;
const int PRESENT_VELOCITY_ADDRESS = 128;
const int PRESENT_POSITION_ADDRESS = 132;

const RUnits::Degrees zero_degree = 180.0;

class Dynamixel_X_Controller : public Actuator_Controller
{
public:
    Dynamixel_X_Controller(const std::string& controller_name,
    const std::string& serial_port, unsigned int baudrate, bool& stop_flag);
    ~Dynamixel_X_Controller();

    void addServo(Dynamixel_X::Actuator_Properties_Ptr actuator);

    virtual void readState() override;
    virtual void writeCommand() override;

    virtual void enableActuators() override;
    virtual void disableActuators() override;

    virtual bool getErrorDetails(std::string& error_msg) override;

    virtual ::Actuator_Properties_Ptr getActuator(const std::string&) override;
    virtual void getActuatorNames(std::vector<std::string>&) override;

protected:
    void _checkResponse(Actuator_Properties_Ptr actuator, int comm_result, uint8_t& dxl_error, bool throw_exception=false);
    void _handleErrorResponse(Actuator_Properties_Ptr actuator, int comm_result, uint8_t& dxl_error);
    void _enableActuators();
    bool _handleHardwareError(Actuator_Properties_Ptr actuator, std::string& error_msg);

    std::string controller_name_;
    std::string serial_port_;

    bool perform_actuator_enable_;
    bool actuator_enabled_status_;

    bool& stop_flag_;
    bool error_status_;
    uint8_t com_error_count_;
    std::string error_description_;

    std::shared_ptr<dynamixel::PortHandler> port_handler_;
    std::shared_ptr<dynamixel::PacketHandler> packet_handler_;
    dynamixel::GroupSyncRead sync_read_;
    dynamixel::GroupSyncWrite sync_write_;

    std::map<std::string, Dynamixel_X::Actuator_Properties_Ptr> servo_map_;
};
}
#endif