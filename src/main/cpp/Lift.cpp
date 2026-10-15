#include "Lift.h"

#include <cmath>


Lift::Lift()
{
    Stop();
}


double Lift::ClampOutput(double speed)
{
    if (speed > Constants::kLiftMaxOutput)
    {
        return Constants::kLiftMaxOutput;
    }

    if (speed < -Constants::kLiftMaxOutput)
    {
        return -Constants::kLiftMaxOutput;
    }

    return speed;
}


void Lift::Set(double speed)
{
    liftMotor.Set(
        ClampOutput(speed)
    );
}


void Lift::Up(double speed)
{
    Set(
        std::abs(speed)
    );
}


void Lift::Down(double speed)
{
    Set(
        -std::abs(speed)
    );
}


void Lift::Stop()
{
    liftMotor.Set(0.0);
}


bool Lift::IsBottomLimitPressed()
{
    // Preserve the behavior used by the tested monolithic code:
    // the Titan bottom-limit input is active when it reads 0.
    return
        liftMotor.GetLimitSwitch(true)
        ==
        0;
}


double Lift::GetRawEncoder()
{
    return
        liftEncoder.GetEncoderDistance();
}


double Lift::GetPosition()
{
    return
        GetRawEncoder()
        -
        zeroEncoderValue;
}


void Lift::ZeroAtCurrentPosition()
{
    zeroEncoderValue =
        GetRawEncoder();

    homed =
        true;
}


bool Lift::IsHomed() const
{
    return homed;
}


void Lift::ClearHomed()
{
    homed =
        false;
}
