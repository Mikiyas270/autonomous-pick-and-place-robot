#pragma once

#include <array>

class TeleopInput {
public:
    static std::array<double, 3> CalculateCommand(
        double leftX,
        double leftY,
        double rightX
    );

private:
    static double ApplyDeadband(double value);
};