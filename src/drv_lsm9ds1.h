#include "mbed.h"
#include "drv_lsm9ds1_types.h"
#include "drv_lsm9ds1_registers.h"

enum lsm9ds1_axis
{
  X_AXIS,
  Y_AXIS,
  Z_AXIS,
  ALL_AXIS
};

class LSM9DS1
{
 public:
  LSM9DS1(uint8_t xgAddr, uint8_t mAddr);
  uint16_t init();
  uint8_t tempAvailable();
  void update();
  float getRoll();
  float getPitch();
  float getHeading();

  int16_t gx, gy, gz; // x, y, and z axis readings of the gyroscope
  int16_t ax, ay, az; // x, y, and z axis readings of the accelerometer
  int16_t mx, my, mz; // x, y, and z axis readings of the magnetometer

 private:
  I2C _i2c;
  IMUSettings settings;

  uint8_t mt, xgt;
  uint8_t _mAddress, _xgAddress;
  int16_t temperature; // Chip temperature
  bool _autoCalc;
  float magSensitivity[4];
  float gRes, aRes, mRes;
  float gBias[3], aBias[3], mBias[3];
  int16_t gBiasRaw[3], aBiasRaw[3], mBiasRaw[3];

  void constrainScales();
  void initGyro();
  void initAccel();
  void initMag();
  void readAccel();
  int16_t readAccel(lsm9ds1_axis axis);
  void readMag();
  int16_t readMag(lsm9ds1_axis axis);
  void readTemp();
  void readGyro();
  int16_t readGyro(lsm9ds1_axis axis);
  void calcgRes();
  void calcaRes();
  void calcmRes();
  float calcGyro(int16_t gyro);
  float calcAccel(int16_t accel);
  float calcMag(int16_t mag);
  uint8_t xgReadByte(uint8_t subAddress);
  void xgReadBytes(uint8_t subAddress, uint8_t *dest, uint8_t count);
  uint8_t mReadByte(uint8_t subAddress);
  void mReadBytes(uint8_t subAddress, uint8_t *dest, uint8_t count);
  void xgWriteByte(uint8_t subAddress, uint8_t data);
  void mWriteByte(uint8_t subAddress, uint8_t data);
  uint8_t gyroAvailable();
  uint8_t magAvailable(lsm9ds1_axis axis = ALL_AXIS);
  uint8_t accelAvailable();
  void calibrate(bool autoCalc = true);
  void calibrateMag(bool loadIn = true);
  void magOffset(uint8_t axis, int16_t offset);
  void enableFIFO(bool enable);
  void setFIFO(fifoMode_type fifoMode, uint8_t fifoThs);
  void I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data);
  uint8_t I2CreadByte(uint8_t address, uint8_t subAddress);
  uint8_t I2CreadBytes(uint8_t address, uint8_t subAddress, uint8_t *dest, uint8_t count);
};
