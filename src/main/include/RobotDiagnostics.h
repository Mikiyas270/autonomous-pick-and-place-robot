#pragma once

#include "DriveTrain.h"
#include "GyroSensor.h"

class RobotDiagnostics {
public:
    RobotDiagnostics(
        DriveTrain& driveTrain,
        GyroSensor& gyro
    );

    void Publish();

private:
    DriveTrain& driveTrain;
    GyroSensor& gyro;
};