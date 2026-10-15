#pragma once

#include <array>

#include <studica/TitanQuad.h>
#include <studica/TitanQuadEncoder.h>

#include "Constants.h"


class DriveTrain
{
public:

    DriveTrain();

    void SetWheelSpeeds(
        const std::array<double, 3>& speeds
    );

    void DriveRobotRelative(
        double vx,
        double vy,
        double omega,
        double maxOutput
    );

    void Stop();


    std::array<double, 3>
        GetWheelDistancesMm();


    double GetFrontRightDistanceMm();

    double GetRearDistanceMm();

    double GetFrontLeftDistanceMm();


private:

    studica::TitanQuad frontRightMotor{
        Constants::kTitanId,
        Constants::kFrontRightMotor
    };


    studica::TitanQuad rearMotor{
        Constants::kTitanId,
        Constants::kRearMotor
    };


    studica::TitanQuad frontLeftMotor{
        Constants::kTitanId,
        Constants::kFrontLeftMotor
    };


    studica::TitanQuadEncoder frontRightEncoder{
        frontRightMotor,
        Constants::kFrontRightMotor,
        Constants::kDriveDistancePerTickMm
    };


    studica::TitanQuadEncoder rearEncoder{
        rearMotor,
        Constants::kRearMotor,
        Constants::kDriveDistancePerTickMm
    };


    studica::TitanQuadEncoder frontLeftEncoder{
        frontLeftMotor,
        Constants::kFrontLeftMotor,
        Constants::kDriveDistancePerTickMm
    };


    static constexpr double kFrontRightSign = +1.0;

    static constexpr double kRearSign = +1.0;

    static constexpr double kFrontLeftSign = +1.0;
};