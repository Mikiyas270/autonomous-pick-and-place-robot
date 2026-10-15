#pragma once

#include <studica/TitanQuad.h>
#include <studica/TitanQuadEncoder.h>

#include "Constants.h"

class Lift
{
public:
    Lift();

    void Set(double speed);
    void Up(double speed);
    void Down(double speed);
    void Stop();

    bool IsBottomLimitPressed();

    double GetRawEncoder();
    double GetPosition();

    void ZeroAtCurrentPosition();

    bool IsHomed() const;
    void ClearHomed();

private:
    studica::TitanQuad liftMotor{
        Constants::kTitanId,
        Constants::kLiftMotor
    };

    studica::TitanQuadEncoder liftEncoder{
        liftMotor,
        Constants::kLiftMotor,
        1.0
    };

    double zeroEncoderValue = 0.0;
    bool homed = false;

    double ClampOutput(double speed);
};
