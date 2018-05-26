//
// Created by Jamie on 24-4-2018.
//

#include <arm_math.h>
#include "usain_imu.h"


UsainIMU::UsainIMU() :
    _imu(0xD6, 0x3C)
{

}

int UsainIMU::init()
{
  uint16_t status = _imu.init();

  if (status == 0)
  {
    return -1;
  }

  _collision_thread.start(callback(this, &UsainIMU::collision_thread));

  return 0;
}

float UsainIMU::get_compass()
{
  return _imu.getHeading();
}

float UsainIMU::get_pitch()
{
  return _imu.getPitch();
}

float UsainIMU::get_roll()
{
  return _imu.getRoll();
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
    _imu.update();

    if (abs(_imu.ax - prev_ax) > 6500 || abs(_imu.ay - prev_ay) > 6500)
    {
      if(_colliion_callback)
        _colliion_callback.call();
    }

    prev_ax = _imu.ax;
    prev_ay = _imu.ay;

    wait(0.01);
  }
}
