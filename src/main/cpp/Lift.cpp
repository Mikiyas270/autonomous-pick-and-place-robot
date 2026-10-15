#include "Lift.h"

#include <cmath>


Lift::Lift() {
    Stop();
}


double Lift::ClampOutput(double speed) {

    if (speed > Constants::kLiftMaxOutput) {
        return Constants::kLiftMaxOutput;
    }

    if (speed < -Constants::kLiftMaxOutput) {
        return -Constants::kLiftMaxOutput;
    }

    return speed;
}


void Lift::Set(double speed) {

    liftMotor.Set(
        ClampOutput(speed)
    );
}


void Lift::Up(double speed) {

    Set(std::abs(speed));
}


void Lift::Down(double speed) {

    Set(-std::abs(speed));
}


void Lift::Stop() {

    liftMotor.Set(0.0);
}