#include <arm_math.h>
#include "mbed.h"

#include "usain_imu.h"

void isr()
{
  printf("collision\n");
}

int main()
{
  UsainIMU imu;

  if(imu.init() < 0)
  {
    error("error: no imu connected\n");
  }

//  imu.register_on_collision(callback(isr));

  while (true)
  {
    imu.get_compass();
    printf("heading: %f\n", imu.get_compass());
    wait(0.1);
  }
}
