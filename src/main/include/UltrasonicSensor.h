#pragma once

#include <frc/Ultrasonic.h>

class UltrasonicSensor
{
public:
    UltrasonicSensor(
        int triggerChannel,
        int echoChannel
    );

    bool IsRangeValid();

    double GetDistanceMillimeters();
    double GetDistanceCentimeters();

private:
    frc::Ultrasonic sensor;
};
