//
// Created by Jamie on 24-4-2018.
//

#include "usain_imu.h"

UsainIMU::UsainIMU() :
        _imu(D14, D15),
        _interrupt(D7)
{
    // gyro.latchInterrupt controls the latching of the
    // gyro and accelerometer interrupts (INT1 and INT2).
    // false = no latching
    _imu.settings.gyro.latchInterrupt = false;

    // Set gyroscope scale to +/-245 dps:
    _imu.settings.gyro.scale = 245;
    // Set gyroscope (and accel) sample rate to 14.9 Hz
    _imu.settings.gyro.sampleRate = 1;
    // Set accelerometer scale to +/-2g
    _imu.settings.accel.scale = 2;
    // Set magnetometer scale to +/- 4g
    _imu.settings.mag.scale = 4;
    // Set magnetometer sample rate to 0.625 Hz
    _imu.settings.mag.sampleRate = 0;

    uint16_t status = _imu.begin();

    if (status == 0)
    {
        error("Error: no IMU connection\n");
    }
}

float UsainIMU::get_compass()
{
    return 0;
}

float UsainIMU::get_pitch()
{

    return 0;
}

float UsainIMU::get_roll()
{
    return 0;
}

void UsainIMU::register_on_collision(const Callback<void()> &callback, int threshold)
{
    _imu.configGyroInt(ZHIE_G, false, false);
    _imu.configGyroThs(threshold, Z_AXIS, 10, true);
    _imu.configInt(XG_INT1, INT1_IG_G | INT_IG_XL, INT_ACTIVE_LOW, INT_PUSH_PULL);

    _interrupt.fall(callback);
}
