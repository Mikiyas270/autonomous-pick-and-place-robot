#pragma once

#include <AHRS.h>
#include <frc/SPI.h>

class GyroSensor {
public:
    GyroSensor();

    double GetYaw();
    double GetAngle();
    double GetRate();

    // Exposes the raw X gyro rate so Robot.cpp can preserve
    // the calibrated integration used on the real robot.
    double GetRawGyroX();

    bool IsConnected();
    bool IsCalibrating();

    void ZeroYaw();

private:
    AHRS navx;
};
