#include "LineFollower.h"


LineFollower::LineFollower()
    : cobra{}
{
}


double LineFollower::GetRawValue(
    int channel
)
{
    return
        cobra.GetRawValue(
            channel
        );
}


float LineFollower::GetVoltage(
    int channel
)
{
    return
        cobra.GetVoltage(
            channel
        );
}