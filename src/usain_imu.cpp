//
// Created by Jamie on 24-4-2018.
//

#include <arm_math.h>
#include "usain_imu.h"

UsainIMU::UsainIMU() :
    _imu(D14, D15)
{

}

int UsainIMU::init()
{
  // gyro.latchInterrupt controls the latching of the
  // gyro and accelerometer interrupts (INT1 and INT2).
  // false = no latching
  _imu.settings.gyro.latchInterrupt = 0;

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
    return -1;
  }

  _collision_thread.start(callback(this, &UsainIMU::collision_thread));

  return 0;
}

float UsainIMU::get_compass()
{
  return 0;
}

float UsainIMU::get_pitch()
{
  float pitch = atan2(-_imu.ax, sqrt(_imu.ay * _imu.ay + _imu.az * _imu.az));
  pitch *= 180.0 / PI;

  return pitch;
}

float UsainIMU::get_roll()
{
  return 0;
}

void UsainIMU::register_on_collision(const Callback<void()> &callback)
{
  _colliion_callback = callback;
}

void UsainIMU::collision_thread()
{
  int prev_ax = 0;
  int prev_ay = 0;

  while (true)
  {
    _imu.readAccel();
    _imu.readGyro();
    _imu.readMag();

//    printf("ax: %d   ay:%d\n", _imu.ax, _imu.ay);

//    if (_imu.ax > 1000 || _imu.ax < -1000 || _imu.ay > 1000 || _imu.ay < -1000)
//    {
//      printf("collision!\n");
//    }

    if(abs(_imu.ax - prev_ax) > 6500 || abs(_imu.ay - prev_ay) > 6500)
    {
      _colliion_callback.call();
    }

    prev_ax = _imu.ax;
    prev_ay = _imu.ay;

    wait(0.05);
  }
}
