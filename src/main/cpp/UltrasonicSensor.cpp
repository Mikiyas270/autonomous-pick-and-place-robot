#include "UltrasonicSensor.h"


UltrasonicSensor::UltrasonicSensor(
    int triggerChannel,
    int echoChannel
)
    : sensor(
          triggerChannel,
          echoChannel
      )
{
    sensor.SetAutomaticMode(
        true
    );
}


double
UltrasonicSensor::GetDistanceMillimeters()
{
    return
        sensor.GetRangeMM();
}


double
UltrasonicSensor::GetDistanceCentimeters()
{
    return
        sensor.GetRangeMM()
        /
        10.0;
}