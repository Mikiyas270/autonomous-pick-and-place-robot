#include "TeleopInput.h"
#include "KiwiDrive.h"
#include "Constants.h"

#include <cmath>

double TeleopInput::ApplyDeadband(double value) {
    if (std::abs(value) < Constants::kJoystickDeadband) {
        return 0.0;
    }

    return value;
}

std::array<double, 3> TeleopInput::CalculateCommand(
    double leftX,
    double leftY,
    double rightX
) {
    double vx = ApplyDeadband(leftX);

    // This sign was calibrated from the real controller.
    double vy = ApplyDeadband(leftY);

    // KiwiDrive convention: +omega = CCW.
    // Negating rightX makes stick-right command clockwise rotation.
    double omega = ApplyDeadband(-rightX);

    auto speeds =
        KiwiDrive::CalculateWheelSpeeds(vx, vy, omega);

    for (double& speed : speeds) {
        speed *= Constants::kTeleopMaxOutput;
    }

    return speeds;
}