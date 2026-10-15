#pragma once

#include <array>


class KiwiDrive
{
public:

    static std::array<double, 3>
        CalculateWheelSpeeds(
            double vx,
            double vy,
            double omega
        );
};