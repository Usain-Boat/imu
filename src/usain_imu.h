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

  int init();

  float get_compass();

  float get_pitch();

  float get_roll();

  void register_on_collision(const Callback<void()> &callback);

 private:
  void collision_thread();

  LSM9DS1 _imu;

  Thread _collision_thread;

  Callback<void()> _colliion_callback;
};

#endif //ACCELEROMETER_USAIN_IMU_H
