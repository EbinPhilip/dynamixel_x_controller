#include "Dynamixel_X_Units.h"

#include <cmath>
#include <stdexcept>

namespace Dynamixel_X
{
template<typename T>
void ConvertToByteArray(T& source, uint8_t* dest_array)
{
    uint32_t data_length = sizeof(source);
    for(int i = 0; i<data_length; ++i)
    {
        uint8_t temp = (source>>(i*8)) & 0xff;
        dest_array[i] = temp;
    }
}
}

using namespace Dynamixel_X;

Pos_Unit::Pos_Unit(RUnits::Degrees position)
{
    position_ = fabs(position.Value())/POSITION_MAX.Value() *  POSITION_MAX_UNITS;
}

Pos_Unit::Pos_Unit(int position)
{
    if (abs(position)<=POSITION_MAX_UNITS)
    {
        position_ = position;
    }
    else
    {
        throw std::runtime_error("position value out of range");
    }
}

int Pos_Unit::Value() const
{
    return position_;
}

void Pos_Unit::ToByteArray(uint8_t* pos_byte_array)
{
    ConvertToByteArray(position_, pos_byte_array);
}

Pos_Unit::operator RUnits::Degrees() const
{
    return RUnits::Degrees((double)position_/POSITION_MAX_UNITS * POSITION_MAX.Value());
}

Speed_Unit::Speed_Unit(RUnits::RPM speed)
{
    speed_ = fabs(speed.Value())/SPEED_UNIT.Value();
}

Speed_Unit::Speed_Unit(int speed)
{
    if (abs(speed)<=SPEED_MAX_UNITS)
    {
        speed_ = abs(speed);
    }
    else
    {
        throw std::runtime_error("speed value out of range");
    }
}

int Speed_Unit::Value() const
{
    return speed_;
}

void Speed_Unit::ToByteArray(uint8_t* pos_byte_array)
{
    ConvertToByteArray(speed_, pos_byte_array);
}

Speed_Unit::operator RUnits::RPM() const
{
    return RUnits::RPM((double)speed_ * SPEED_UNIT.Value());
}