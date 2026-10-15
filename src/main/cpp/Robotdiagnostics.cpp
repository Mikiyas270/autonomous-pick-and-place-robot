#include "RobotDiagnostics.h"

#include <frc/smartdashboard/SmartDashboard.h>

RobotDiagnostics::RobotDiagnostics(
    DriveTrain& driveTrain,
    GyroSensor& gyro
)
    : driveTrain(driveTrain),
      gyro(gyro)
{
}

void RobotDiagnostics::Publish() {

    // ========================================================
    // DRIVE ENCODERS
    // ========================================================

    frc::SmartDashboard::PutNumber(
        "Encoder/Front Right Distance mm",
        driveTrain.GetFrontRightDistanceMm()
    );

    frc::SmartDashboard::PutNumber(
        "Encoder/Rear Distance mm",
        driveTrain.GetRearDistanceMm()
    );

    frc::SmartDashboard::PutNumber(
        "Encoder/Front Left Distance mm",
        driveTrain.GetFrontLeftDistanceMm()
    );


    // ========================================================
    // VMX / navX GYRO
    // ========================================================

    frc::SmartDashboard::PutNumber(
        "Gyro/Yaw deg",
        gyro.GetYaw()
    );

    frc::SmartDashboard::PutNumber(
        "Gyro/Angle deg",
        gyro.GetAngle()
    );

    frc::SmartDashboard::PutNumber(
        "Gyro/Rate deg per sec",
        gyro.GetRate()
    );

    frc::SmartDashboard::PutBoolean(
        "Gyro/Connected",
        gyro.IsConnected()
    );

    frc::SmartDashboard::PutBoolean(
        "Gyro/Calibrating",
        gyro.IsCalibrating()
    );
}