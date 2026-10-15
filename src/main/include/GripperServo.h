#pragma once

#include <studica/Servo.h>

class GripperServo {
public:

    explicit GripperServo(int pwmChannel);

    void SetAngle(double angle);

private:

    studica::Servo servo;
};