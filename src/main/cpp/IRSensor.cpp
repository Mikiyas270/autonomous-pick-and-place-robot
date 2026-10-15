#include "IRSensor.h"


IRSensor::IRSensor(
    int analogChannel
)
    : sensor(
          analogChannel
      )
{
}


int IRSensor::GetRawValue()
{
    return
        sensor.GetValue();
}


double IRSensor::GetVoltage()
{
    return
        sensor.GetVoltage();
}