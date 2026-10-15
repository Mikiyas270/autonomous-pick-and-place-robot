#pragma once

#include <AHRS.h>
#include <frc/SPI.h>

class GyroSensor {
public:
    GyroSensor();

    double GetYaw();
    double GetAngle();
    double GetRate();

    bool IsConnected();
    bool IsCalibrating();

    void ZeroYaw();

private:
    AHRS navx;
};