#pragma once

#include <studica/Cobra.h>


class LineFollower
{
public:

    LineFollower();

    double GetRawValue(
        int channel
    );

    float GetVoltage(
        int channel
    );


private:

    studica::Cobra cobra;
};