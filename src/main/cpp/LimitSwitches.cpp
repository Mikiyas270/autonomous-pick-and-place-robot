#include "LimitSwitches.h"

LimitSwitches::LimitSwitches(
    int bottomChannel,
    int topChannel
)
    : bottomLimit(bottomChannel),
      topLimit(topChannel)
{
}

bool LimitSwitches::GetBottomRaw() const {
    return bottomLimit.Get();
}

bool LimitSwitches::GetTopRaw() const {
    return topLimit.Get();
}