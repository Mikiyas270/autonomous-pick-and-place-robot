#include "KiwiDrive.h"

#include <algorithm>
#include <array>
#include <cmath>


std::array<double, 3>
KiwiDrive::CalculateWheelSpeeds(
    double vx,
    double vy,
    double omega
)
{
    constexpr double PI =
        3.14159265358979323846;


    constexpr std::array<double, 3>
        wheelAnglesDeg{
            0.0,
            120.0,
            240.0
        };


    std::array<double, 3>
        wheelSpeeds{};


    for (int i = 0; i < 3; ++i)
    {
        const double angle =
            wheelAnglesDeg[i]
            *
            PI
            /
            180.0;


        wheelSpeeds[i] =
            -vx * std::cos(angle)
            +
            vy * std::sin(angle)
            -
            omega;
    }


    double maximum = 1.0;


    for (double speed : wheelSpeeds)
    {
        maximum =
            std::max(
                maximum,
                std::abs(speed)
            );
    }


    for (double& speed : wheelSpeeds)
    {
        speed /= maximum;
    }


    return wheelSpeeds;
}