#include "Dynamixel_X_Units.h"
#include "rotational_units/Rotational_Units.h"

#include <iostream>
#include <stdint.h>

using namespace Dynamixel_X;
using namespace RUnits;
using namespace std;

int main(void)
{
    double position, speed;
    cin>>position>>speed;
    uint8_t array[4] = {0};

    Degrees deg = position;
    Pos_Unit pos = deg;
    pos.ToByteArray(array);
    cout<<pos.Value()<<endl;
    cout<<((Degrees)pos).Value()<<endl;
    for(int i = 3; i>=0;--i)
    {
        cout<<(int)array[i]<<" ";
    }
    cout<<endl<<endl;

    RPM rpm = speed;
    Speed_Unit sp = rpm;
    sp.ToByteArray(array);
    cout<<sp.Value()<<endl;
    cout<<((Degrees)pos).Value()<<endl;
    for(int i = 3; i>=0;--i)
    {
        cout<<(int)array[i]<<" ";
    }
    cout<<endl;

    return 0;
}