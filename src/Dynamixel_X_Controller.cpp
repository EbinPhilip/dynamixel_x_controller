#include "Dynamixel_X_Controller.h"

#include <rotational_units/Rotational_Units.h>
#include "Dynamixel_X_Units.h"

#include <ros/console.h>

#include <stdexcept>
#include <cmath>

using namespace dynamixel;
using namespace Dynamixel_X;

Dynamixel_X_Controller::Dynamixel_X_Controller(const std::string& controller_name,
    const std::string& serial_port, unsigned int baudrate, bool& stop_flag)
    : controller_name_(controller_name),
      serial_port_(serial_port),
      perform_actuator_enable_(false),
      actuator_enabled_status_(false),
      stop_flag_(stop_flag),
      error_status_(false),
      com_error_count_(0),
      port_handler_(std::shared_ptr<PortHandler>(PortHandler::getPortHandler(serial_port.c_str()))),
      packet_handler_(std::shared_ptr<PacketHandler>(PacketHandler::getPacketHandler(2.0))),
      sync_read_(GroupSyncRead(port_handler_.get(), packet_handler_.get(), PRESENT_LOAD_ADDRESS, 10)),
      sync_write_(GroupSyncWrite(port_handler_.get(), packet_handler_.get(), PROFILE_ACCELERATION, 12))
{
    if(!port_handler_->openPort())
    {
        throw std::runtime_error("Dynamixel X Controller: Failed to open port");
    }
    if(!port_handler_->setBaudRate(baudrate))
    {
        throw std::runtime_error("Dynamixel X Controller: Failed to set baudrate");
    }
}

Dynamixel_X_Controller::~Dynamixel_X_Controller()
{
    if (actuator_enabled_status_)
    {
        try
        {
            disableActuators();
        }
        catch(...)
        {
            ROS_ERROR("%s: unexpected error in desctructor", controller_name_.c_str());
        }
    }
    port_handler_->closePort();
}

void Dynamixel_X_Controller::addServo(Actuator_Properties_Ptr actuator)
{
    uint16_t dxl_model_number;
    uint8_t dxl_error;
    _handleErrorResponse(packet_handler_->ping(port_handler_.get(),
                                            actuator->servo_id, &dxl_model_number, &dxl_error),
                                            dxl_error);
    Pos_Unit ccw_limit = RUnits::Degrees(actuator->ccw_limit_deg.Value() + zero_degree.Value());
    Pos_Unit cw_limit = RUnits::Degrees(actuator->cw_limit_deg.Value() + zero_degree.Value());

    _handleErrorResponse(packet_handler_->write1ByteTxRx(port_handler_.get(),
                                            actuator->servo_id, TORQUE_ENABLE_ADDRESS,
                                            0, &dxl_error),
                                            dxl_error);
    _handleErrorResponse(packet_handler_->write4ByteTxRx(port_handler_.get(),
                                            actuator->servo_id, MAX_POSITION_LIMIT_ADDRESS,
                                            ccw_limit.Value(), &dxl_error),
                                            dxl_error);
    _handleErrorResponse(packet_handler_->write4ByteTxRx(port_handler_.get(),
                                            actuator->servo_id, MIN_POSITION_LIMIT_ADDRESS,
                                            cw_limit.Value(), &dxl_error),
                                            dxl_error);
    
    servo_map_.insert(std::make_pair(actuator->actuator_name, actuator));
    sync_read_.addParam(actuator->servo_id);
}

void Dynamixel_X_Controller::readState()
{
    if (stop_flag_ || error_status_)
    {
        return;
    }

    int com_result = sync_read_.txRxPacket();
    if (com_result != COMM_SUCCESS)
    {
        com_error_count_++;
        if (com_error_count_ >= 3)
        {
            error_description_ = packet_handler_->getTxRxResult(com_result);
            stop_flag_ = true;
            error_status_ = true;
        }
        return;
    }
    else
    {
        com_error_count_ = 0;
    }

    for (auto& servo : servo_map_)
    {
        uint8_t dxl_error = 0;
        if (sync_read_.getError(servo.second->servo_id, &dxl_error))
        {
            error_description_ = packet_handler_->getRxPacketError(dxl_error);
            stop_flag_ = true;
            error_status_ = true;
            return;
        }
        else if (!sync_read_.isAvailable(servo.second->servo_id, PRESENT_LOAD_ADDRESS, 10))
        {
            servo.second->bad_response_count++;
            if (servo.second->bad_response_count >= 3)
            {
                error_description_ = servo.second->actuator_type+std::to_string(servo.second->servo_id)+": sync read failed!";
                stop_flag_ = true;
                error_status_ = true;
                return;
            }
            continue;
        }
        else
        {
            servo.second->bad_response_count = 0;
            servo.second->state.effort = sync_read_.getData(servo.second->servo_id, PRESENT_LOAD_ADDRESS, 2);
            Pos_Unit pos = sync_read_.getData(servo.second->servo_id, PRESENT_POSITION_ADDRESS, 4);
            RUnits::Degrees pos_deg = pos;
            RUnits::RPM rpm = Speed_Unit(sync_read_.getData(servo.second->servo_id, PRESENT_VELOCITY_ADDRESS, 4));
            
            pos_deg = pos_deg.Value() - zero_degree.Value();
            
            double position = static_cast<RUnits::Radians>(pos_deg).Value();
            double speed = static_cast<RUnits::Radians_Per_Sec>(rpm).Value();
            if ((position - servo.second->state.position) < 0)
            {
                speed*=-1;
            }
            servo.second->state.position = position;
            servo.second->state.velocity = speed;
        }
    }
}

void Dynamixel_X_Controller::writeCommand()
{
    if (perform_actuator_enable_)
    {
        if (!actuator_enabled_status_)
            _enableActuators();
        perform_actuator_enable_ = false;
    }

    if (stop_flag_ || error_status_ || !actuator_enabled_status_)
    {
        return;
    }

    for (auto& servo : servo_map_)
    {
        uint8_t packet[12] = {0};
        RUnits::Degrees pos_deg = RUnits::Radians(servo.second->command.position);
        RUnits::RPM speed_rpm = RUnits::Radians_Per_Sec(fabs(servo.second->command.velocity));
        
        pos_deg = zero_degree.Value() + pos_deg.Value();

        Pos_Unit pos = pos_deg;
        Speed_Unit speed = speed_rpm;

        speed.ToByteArray(packet + 4);
        pos.ToByteArray(packet + 8);

        sync_write_.addParam(servo.second->servo_id, packet);
    }
    int dxl_comm_result = sync_write_.txPacket();
    sync_write_.clearParam();
}

void Dynamixel_X_Controller::enableActuators()
{
    if (!actuator_enabled_status_)
    {
        perform_actuator_enable_ = true;
    }
}

void Dynamixel_X_Controller::disableActuators()
{
    try
    {
        for (auto& servo : servo_map_)
        {
            uint8_t dxl_error = 0;
            _handleErrorResponse(packet_handler_->write1ByteTxRx(port_handler_.get(),
                                                servo.second->servo_id, TORQUE_ENABLE_ADDRESS,
                                                0, &dxl_error),
                                                dxl_error);
            _handleErrorResponse(packet_handler_->write1ByteTxRx(port_handler_.get(),
                                                servo.second->servo_id, LED_ADDRESS,
                                                0, &dxl_error),
                                                dxl_error);
        }
    }
    catch (std::exception& e)
    {
        std::string error_msg = e.what();
        ROS_ERROR("%s", error_msg.c_str());
    }
    catch (...)
    {
        ROS_ERROR("%s: unexpected error, disable failed", controller_name_.c_str());
    }
    
    actuator_enabled_status_ = false;
}

bool Dynamixel_X_Controller::getErrorDetails(std::string& error_msg)
{
    if (error_status_)
    {
        error_msg = error_description_;
    }
    return error_status_;
}

::Actuator_Properties_Ptr Dynamixel_X_Controller::getActuator(const std::string& name)
{
    auto it = servo_map_.find(name);
    if (it != servo_map_.end())
    {
        return (it->second);
    }
    else
    {
        return nullptr;
    }
}

void Dynamixel_X_Controller::getActuatorNames(std::vector<std::string>& names)
{
    for (auto it : servo_map_)
    {
        names.push_back(it.first);
    }
}

void Dynamixel_X_Controller::_checkResponse(int comm_result, uint8_t& dxl_error, bool throw_exception)
{
    std::string error;
    if (comm_result != COMM_SUCCESS)
    {
        error = packet_handler_->getTxRxResult(comm_result);
    }
    else if (dxl_error)
    {
        error = packet_handler_->getRxPacketError(dxl_error);
    }
    else
    {
        return;
    }
    dxl_error = 0;
    if (throw_exception)
    {
        throw std::runtime_error(error);
    }
}

void Dynamixel_X_Controller::_handleErrorResponse(int comm_result, uint8_t& dxl_error)
{
    return _checkResponse(comm_result, dxl_error, true);
}

void Dynamixel_X_Controller::_enableActuators()
{
    if (stop_flag_ || error_status_)
    {
        return;
    }

    for (auto& servo : servo_map_)
    {
        uint8_t dxl_error = 0;
        _handleErrorResponse(packet_handler_->write1ByteTxRx(port_handler_.get(),
                                            servo.second->servo_id, TORQUE_ENABLE_ADDRESS,
                                            1, &dxl_error),
                                            dxl_error);
        _handleErrorResponse(packet_handler_->write1ByteTxRx(port_handler_.get(),
                                            servo.second->servo_id, LED_ADDRESS,
                                            1, &dxl_error),
                                            dxl_error);
    }
    actuator_enabled_status_ = true;
}
