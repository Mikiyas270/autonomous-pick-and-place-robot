#pragma once

#include <frc/DigitalInput.h>

class LimitSwitches {
public:
    LimitSwitches(int bottomChannel, int topChannel);

    bool GetBottomRaw() const;
    bool GetTopRaw() const;

private:
    frc::DigitalInput bottomLimit;
    frc::DigitalInput topLimit;
};