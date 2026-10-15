#include "GripperServo.h"

GripperServo::GripperServo(int pwmChannel)
    : servo(pwmChannel)
{
}

void GripperServo::SetAngle(double angle)
{
    if (angle < 0.0)
    {
        angle = 0.0;
    }

    if (angle > 300.0)
    {
        angle = 300.0;
    }

    servo.SetAngle(angle);
}