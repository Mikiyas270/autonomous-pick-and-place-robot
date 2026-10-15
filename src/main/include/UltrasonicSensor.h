#pragma once

#include <frc/Ultrasonic.h>


class UltrasonicSensor
{
public:

    UltrasonicSensor(
        int triggerChannel,
        int echoChannel
    );

    double GetDistanceMillimeters();

    double GetDistanceCentimeters();


private:

    frc::Ultrasonic sensor;
};