#include "GyroSensor.h"

GyroSensor::GyroSensor()
    : navx(frc::SPI::Port::kMXP)
{
}

double GyroSensor::GetYaw() {
    return navx.GetYaw();
}

double GyroSensor::GetAngle() {
    return navx.GetAngle();
}

double GyroSensor::GetRate() {
    return navx.GetRate();
}

double GyroSensor::GetRawGyroX() {
    return navx.GetRawGyroX();
}

bool GyroSensor::IsConnected() {
    return navx.IsConnected();
}

bool GyroSensor::IsCalibrating() {
    return navx.IsCalibrating();
}

void GyroSensor::ZeroYaw() {
    navx.ZeroYaw();
}
