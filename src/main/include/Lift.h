#pragma once

#include <studica/TitanQuad.h>

#include "Constants.h"

class Lift {
public:
    Lift();

    void Set(double speed);

    void Up(double speed);
    void Down(double speed);

    void Stop();

private:
    studica::TitanQuad liftMotor{
        Constants::kTitanId,
        Constants::kLiftMotor
    };

    double ClampOutput(double speed);
};