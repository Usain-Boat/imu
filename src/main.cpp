#include <arm_math.h>
#include "mbed.h"

#include "usain_imu.h"

// refresh time. set to 500 for part 2 and 50 for part 4
#define REFRESH_TIME_MS 50

void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
    float roll = atan2(ay, az);
    float pitch = atan2(-ax, sqrt(ay * ay + az * az));

    // Calculate heading
    float heading = atan2(my, mx);

    // Set declination angle on your location and fix heading
    // You can find your declination on: http://magnetic-declination.com/
    // (+) Positive or (-) for negative
    // For Bytom / Poland declination angle is 4'26E (positive)
    // Formula: (deg + (min / 60.0)) / (180 / M_PI);
    float declinationAngle = (1.0 + (35.0 / 60.0)) / (180 / M_PI);
    heading += declinationAngle;

    // Correct for heading < 0deg and heading > 360deg
    if (heading < 0)
    {
        heading += 2 * PI;
    }

    if (heading > 2 * PI)
    {
        heading -= 2 * PI;
    }

    // Convert everything from radians to degrees:
    heading *= 180.0 / PI;
    pitch *= 180.0 / PI;
    roll  *= 180.0 / PI;

//    printf("pitch: %f\n", pitch);
//    printf("roll: %f\n", roll);
    printf("heading: %f\n", heading);
}

void isr()
{
    printf("t\n");
}

int main()
{
    UsainIMU imu;

    imu.register_on_collision(callback(isr), 500);

    while (true)
    {
//        imu.readAccel();
//        imu.readGyro();
//        imu.readMag();
//
//        printf("Ax: %2f\r\n", imu.ax * 9.81);
//        printf("G: %2f, %2f, %2f\r\n", imu.gx, imu.gy, imu.gz);
//        printf("M: %2f, %2f, %2f\r\n\r\n", imu.mx, imu.my, imu.mz);
//
//        printAttitude(imu.ax, imu.ay, imu.az, imu.mx, imu.my, imu.mz);

        wait_ms(REFRESH_TIME_MS);
    }
}
