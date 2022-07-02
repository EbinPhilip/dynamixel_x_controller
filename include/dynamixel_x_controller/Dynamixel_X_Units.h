#ifndef __DYNAMIXEL_X_UNITS_H__
#define __DYNAMIXEL_X_UNITS_H__

#include <rotational_units/Rotational_Units.h>
#include <stdint.h>

namespace Dynamixel_X
{

const int POSITION_MAX_UNITS = 4095;
const int SPEED_MAX_UNITS = 1023; // 1023 is the maximum possible speed value
const RUnits::RPM SPEED_UNIT = 0.229; // 0.229: Dynamixel speed unit
const RUnits::Degrees POSITION_MAX = RUnits::DEGREES_MAX;

class Pos_Unit
{
public:
    Pos_Unit(RUnits::Degrees position);
    Pos_Unit(int position);
    int Value() const;
    void ToByteArray(uint8_t* pos_byte_array);
    operator RUnits::Degrees() const;
protected:
    int position_;
};

class Speed_Unit
{
public:
    Speed_Unit(RUnits::RPM speed);
    Speed_Unit(int speed);
    int Value() const;
    void ToByteArray(uint8_t* pos_byte_array);
    operator RUnits::RPM() const;
protected:
    int speed_;
};

}

#endif