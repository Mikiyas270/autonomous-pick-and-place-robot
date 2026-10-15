#pragma once

#include <frc/AnalogInput.h>


class IRSensor
{
public:

    explicit IRSensor(
        int analogChannel
    );

    int GetRawValue();

    double GetVoltage();


private:

    frc::AnalogInput sensor;
};