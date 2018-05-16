//
// Created by Jamie on 24-4-2018.
//

#ifndef ACCELEROMETER_USAIN_IMU_H
#define ACCELEROMETER_USAIN_IMU_H

#include <mbed.h>

#include "drv_lsm9ds1.h"

class UsainIMU
{
public:
    UsainIMU();

    float get_compass();

    float get_pitch();

    float get_roll();

    void register_on_collision(const Callback<void()> &callback, int threshold);

private:
    LSM9DS1 _imu;

    EventQueue _ev;

    InterruptIn _interrupt;
};

#endif //ACCELEROMETER_USAIN_IMU_H
