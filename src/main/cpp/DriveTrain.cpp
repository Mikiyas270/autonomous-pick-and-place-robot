#include "DriveTrain.h"
#include "KiwiDrive.h"


DriveTrain::DriveTrain()
{
    Stop();
}


void DriveTrain::SetWheelSpeeds(
    const std::array<double, 3>& speeds
)
{
    rearMotor.Set(
        kRearSign * speeds[0]
    );

    frontRightMotor.Set(
        kFrontRightSign * speeds[1]
    );

    frontLeftMotor.Set(
        kFrontLeftSign * speeds[2]
    );
}


void DriveTrain::DriveRobotRelative(
    double vx,
    double vy,
    double omega,
    double maxOutput
)
{
    auto speeds =
        KiwiDrive::CalculateWheelSpeeds(
            vx,
            vy,
            omega
        );


    for (double& speed : speeds)
    {
        speed *= maxOutput;
    }


    SetWheelSpeeds(
        speeds
    );
}


void DriveTrain::Stop()
{
    rearMotor.Set(0.0);

    frontRightMotor.Set(0.0);

    frontLeftMotor.Set(0.0);
}


std::array<double, 3>
DriveTrain::GetWheelDistancesMm()
{
    return
    {
        rearEncoder.GetEncoderDistance(),

        frontRightEncoder.GetEncoderDistance(),

        frontLeftEncoder.GetEncoderDistance()
    };
}


double DriveTrain::GetFrontRightDistanceMm()
{
    return
        frontRightEncoder.
            GetEncoderDistance();
}


double DriveTrain::GetRearDistanceMm()
{
    return
        rearEncoder.
            GetEncoderDistance();
}


double DriveTrain::GetFrontLeftDistanceMm()
{
    return
        frontLeftEncoder.
            GetEncoderDistance();
}