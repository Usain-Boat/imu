//
// Created by Jamie on 23-4-2018.
//

#include "drv_lsm9ds1.h"
#include "math.h"

#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif

LSM9DS1::LSM9DS1(uint8_t xgAddr, uint8_t mAddr)
    : gx(0),
      gy(0),
      gz(0),
      ax(0),
      ay(0),
      az(0),
      mx(0),
      my(0),
      mz(0),
      temperature(0),
      roll(0),
      pitch(0),
      heading(0),
      _i2c(D14, D15)
{
	magSensitivity[0] = 0.00014;
	magSensitivity[1] = 0.00029;
	magSensitivity[2] = 0.00043;
	magSensitivity[3] = 0.00058;
	
  settings.device.
      commInterface = IMU_MODE_I2C;
  settings.device.
      agAddress = xgAddr;
  settings.device.
      mAddress = mAddr;

  settings.gyro.
      enabled = true;
  settings.gyro.
      enableX = true;
  settings.gyro.
      enableY = true;
  settings.gyro.
      enableZ = true;
// gyro scale can be 245, 500, or 2000
  settings.gyro.
      scale = 245;
// gyro sample rate: value between 1-6
// 1 = 14.9    4 = 238
// 2 = 59.5    5 = 476
// 3 = 119     6 = 952
  settings.gyro.
      sampleRate = 6;
// gyro cutoff frequency: value between 0-3
// Actual value of cutoff frequency depends
// on sample rate.
  settings.gyro.
      bandwidth = 0;
  settings.gyro.
      lowPowerEnable = false;
  settings.gyro.
      HPFEnable = false;
// Gyro HPF cutoff frequency: value between 0-9
// Actual value depends on sample rate. Only applies
// if gyroHPFEnable is true.
  settings.gyro.
      HPFCutoff = 0;
  settings.gyro.
      flipX = false;
  settings.gyro.
      flipY = false;
  settings.gyro.
      flipZ = false;
  settings.gyro.
      orientation = 0;
  settings.gyro.
      latchInterrupt = true;

  settings.accel.
      enabled = true;
  settings.accel.
      enableX = true;
  settings.accel.
      enableY = true;
  settings.accel.
      enableZ = true;
// accel scale can be 2, 4, 8, or 16
  settings.accel.
      scale = 2;
// accel sample rate can be 1-6
// 1 = 10 Hz    4 = 238 Hz
// 2 = 50 Hz    5 = 476 Hz
// 3 = 119 Hz   6 = 952 Hz
  settings.accel.
      sampleRate = 6;
// Accel cutoff freqeuncy can be any value between -1 - 3.
// -1 = bandwidth determined by sample rate
// 0 = 408 Hz   2 = 105 Hz
// 1 = 211 Hz   3 = 50 Hz
  settings.accel.
      bandwidth = -1;
  settings.accel.
      highResEnable = false;
// accelHighResBandwidth can be any value between 0-3
// LP cutoff is set to a factor of sample rate
// 0 = ODR/50    2 = ODR/9
// 1 = ODR/100   3 = ODR/400
  settings.accel.
      highResBandwidth = 0;

  settings.mag.
      enabled = true;
// mag scale can be 4, 8, 12, or 16
  settings.mag.
      scale = 4;
// mag data rate can be 0-7
// 0 = 0.625 Hz  4 = 10 Hz
// 1 = 1.25 Hz   5 = 20 Hz
// 2 = 2.5 Hz    6 = 40 Hz
// 3 = 5 Hz      7 = 80 Hz
  settings.mag.
      sampleRate = 7;
  settings.mag.
      tempCompensationEnable = false;
// magPerformance can be any value between 0-3
// 0 = Low power mode      2 = high performance
// 1 = medium performance  3 = ultra-high performance
  settings.mag.
      XYPerformance = 3;
  settings.mag.
      ZPerformance = 3;
  settings.mag.
      lowPowerEnable = false;
// magOperatingMode can be 0-2
// 0 = continuous conversion
// 1 = single-conversion
// 2 = power down
  settings.mag.
      operatingMode = 0;

  settings.temp.
      enabled = true;
  for (
      int i = 0;
      i < 3; i++)
  {
    gBias[i] = 0;
    aBias[i] = 0;
    mBias[i] = 0;
    gBiasRaw[i] = 0;
    aBiasRaw[i] = 0;
    mBiasRaw[i] = 0;
  }
  _autoCalc = false;

}

uint16_t LSM9DS1::init()
{
  //! Todo: don't use _xgAddress or _mAddress, duplicating memory
  _xgAddress = settings.device.agAddress;
  _mAddress = settings.device.mAddress;

  constrainScales();
  // Once we have the scale values, we can calculate the resolution
  // of each sensor. That's what these functions are for. One for each sensor
  calcgRes(); // Calculate DPS / ADC tick, stored in gRes variable
  calcmRes(); // Calculate Gs / ADC tick, stored in mRes variable
  calcaRes(); // Calculate g / ADC tick, stored in aRes variable

  // To verify communication, we can read from the WHO_AM_I register of
  // each device. Store those in a variable so we can return them.
  uint8_t mTest = I2CreadByte(_mAddress, WHO_AM_I_M);      // Read the gyro WHO_AM_I
  mt = mTest;
//	    osDelay(100);
  wait_ms(100);            //
  uint8_t xgTest = I2CreadByte(_xgAddress, WHO_AM_I_XG);   // Read the accel/mag WHO_AM_I
  xgt = xgTest;
  //pc.printf("%x, %x, %x, %x\n\r", mTest, xgTest, _xgAddress, _mAddress);
  uint16_t whoAmICombined = (xgTest << 8) | mTest;

  if (whoAmICombined != ((WHO_AM_I_AG_RSP << 8) | WHO_AM_I_M_RSP))
    return 0;

  // Gyro initialization stuff:
  initGyro(); // This will "turn on" the gyro. Setting up interrupts, etc.
  //osDelay(500);
  // Accelerometer initialization stuff:
  initAccel(); // "Turn on" all axes of the accel. Set up interrupts, etc.
  // osDelay(500);
  // Magnetometer initialization stuff:
  initMag(); // "Turn on" all axes of the mag. Set up interrupts, etc.
  // osDelay(500);
  // Once everything is initialized, return the WHO_AM_I registers we read:
  calibrate();
  calibrateMag();

  return whoAmICombined;
}

void LSM9DS1::constrainScales()
{
  if ((settings.gyro.scale != 245) && (settings.gyro.scale != 500) &&
      (settings.gyro.scale != 2000))
  {
    settings.gyro.scale = 245;
  }

  if ((settings.accel.scale != 2) && (settings.accel.scale != 4) &&
      (settings.accel.scale != 8) && (settings.accel.scale != 16))
  {
    settings.accel.scale = 2;
  }

  if ((settings.mag.scale != 4) && (settings.mag.scale != 8) &&
      (settings.mag.scale != 12) && (settings.mag.scale != 16))
  {
    settings.mag.scale = 4;
  }
}

void LSM9DS1::calcgRes()
{
  gRes = ((float) settings.gyro.scale) / 32768.0;
}

void LSM9DS1::calcaRes()
{
  aRes = ((float) settings.accel.scale) / 32768.0;
}

void LSM9DS1::calcmRes()
{
  //mRes = ((float) settings.mag.scale) / 32768.0;
  switch (settings.mag.scale)
  {
    case 4:mRes = magSensitivity[0];
      break;
    case 8:mRes = magSensitivity[1];
      break;
    case 12:mRes = magSensitivity[2];
      break;
    case 16:mRes = magSensitivity[3];
      break;
  }

}

void LSM9DS1::initGyro()
{
  uint8_t tempRegValue = 0;

  // CTRL_REG1_G (Default value: 0x00)
  // [ODR_G2][ODR_G1][ODR_G0][FS_G1][FS_G0][0][BW_G1][BW_G0]
  // ODR_G[2:0] - Output data rate selection
  // FS_G[1:0] - Gyroscope full-scale selection
  // BW_G[1:0] - Gyroscope bandwidth selection

  // To disable gyro, set sample rate bits to 0. We'll only set sample
  // rate if the gyro is enabled.
  if (settings.gyro.enabled)
  {
    tempRegValue = (settings.gyro.sampleRate & 0x07) << 5;
  }
  switch (settings.gyro.scale)
  {
    case 500:tempRegValue |= (0x1 << 3);
      break;
    case 2000:tempRegValue |= (0x3 << 3);
      break;
      // Otherwise we'll set it to 245 dps (0x0 << 4)
  }
  tempRegValue |= (settings.gyro.bandwidth & 0x3);
  I2CwriteByte(_xgAddress, CTRL_REG1_G, tempRegValue);
  wait_ms(1);
  // CTRL_REG2_G (Default value: 0x00)
  // [0][0][0][0][INT_SEL1][INT_SEL0][OUT_SEL1][OUT_SEL0]
  // INT_SEL[1:0] - INT selection configuration
  // OUT_SEL[1:0] - Out selection configuration
  I2CwriteByte(_xgAddress, CTRL_REG2_G, 0x00);
  wait_ms(1);
  // CTRL_REG3_G (Default value: 0x00)
  // [LP_mode][HP_EN][0][0][HPCF3_G][HPCF2_G][HPCF1_G][HPCF0_G]
  // LP_mode - Low-power mode enable (0: disabled, 1: enabled)
  // HP_EN - HPF enable (0:disabled, 1: enabled)
  // HPCF_G[3:0] - HPF cutoff frequency
  tempRegValue = settings.gyro.lowPowerEnable ? (1 << 7) : 0;
  if (settings.gyro.HPFEnable)
  {
    tempRegValue |= (1 << 6) | (settings.gyro.HPFCutoff & 0x0F);
  }
  I2CwriteByte(_xgAddress, CTRL_REG3_G, tempRegValue);
  //wait_ms(1);
  // CTRL_REG4 (Default value: 0x38)
  // [0][0][Zen_G][Yen_G][Xen_G][0][LIR_XL1][4D_XL1]
  // Zen_G - Z-axis output enable (0:disable, 1:enable)
  // Yen_G - Y-axis output enable (0:disable, 1:enable)
  // Xen_G - X-axis output enable (0:disable, 1:enable)
  // LIR_XL1 - Latched interrupt (0:not latched, 1:latched)
  // 4D_XL1 - 4D option on interrupt (0:6D used, 1:4D used)
  tempRegValue = 0;
  if (settings.gyro.enableZ) tempRegValue |= (1 << 5);
  if (settings.gyro.enableY) tempRegValue |= (1 << 4);
  if (settings.gyro.enableX) tempRegValue |= (1 << 3);
  if (settings.gyro.latchInterrupt) tempRegValue |= (1 << 1);
  I2CwriteByte(_xgAddress, CTRL_REG4, tempRegValue);
  //wait_ms(5);
  // ORIENT_CFG_G (Default value: 0x00)
  // [0][0][SignX_G][SignY_G][SignZ_G][Orient_2][Orient_1][Orient_0]
  // SignX_G - Pitch axis (X) angular rate sign (0: positive, 1: negative)
  // Orient [2:0] - Directional user orientation selection
  tempRegValue = 0;
  if (settings.gyro.flipX) tempRegValue |= (1 << 5);
  if (settings.gyro.flipY) tempRegValue |= (1 << 4);
  if (settings.gyro.flipZ) tempRegValue |= (1 << 3);
  I2CwriteByte(_xgAddress, ORIENT_CFG_G, tempRegValue);
  // wait_ms(5);
}

void LSM9DS1::initAccel()
{
  uint8_t tempRegValue = 0;

  //  CTRL_REG5_XL (0x1F) (Default value: 0x38)
  //  [DEC_1][DEC_0][Zen_XL][Yen_XL][Zen_XL][0][0][0]
  //  DEC[0:1] - Decimation of accel data on OUT REG and FIFO.
  //      00: None, 01: 2 samples, 10: 4 samples 11: 8 samples
  //  Zen_XL - Z-axis output enabled
  //  Yen_XL - Y-axis output enabled
  //  Xen_XL - X-axis output enabled
  if (settings.accel.enableZ) tempRegValue |= (1 << 5);
  if (settings.accel.enableY) tempRegValue |= (1 << 4);
  if (settings.accel.enableX) tempRegValue |= (1 << 3);

  I2CwriteByte(_xgAddress, CTRL_REG5_XL, tempRegValue);

  // CTRL_REG6_XL (0x20) (Default value: 0x00)
  // [ODR_XL2][ODR_XL1][ODR_XL0][FS1_XL][FS0_XL][BW_SCAL_ODR][BW_XL1][BW_XL0]
  // ODR_XL[2:0] - Output data rate & power mode selection
  // FS_XL[1:0] - Full-scale selection
  // BW_SCAL_ODR - Bandwidth selection
  // BW_XL[1:0] - Anti-aliasing filter bandwidth selection
  tempRegValue = 0;
  // To disable the accel, set the sampleRate bits to 0.
  if (settings.accel.enabled)
  {
    tempRegValue |= (settings.accel.sampleRate & 0x07) << 5;
  }
  switch (settings.accel.scale)
  {
    case 4:tempRegValue |= (0x2 << 3);
      break;
    case 8:tempRegValue |= (0x3 << 3);
      break;
    case 16:tempRegValue |= (0x1 << 3);
      break;
      // Otherwise it'll be set to 2g (0x0 << 3)
  }
  if (settings.accel.bandwidth >= 0)
  {
    tempRegValue |= (1 << 2); // Set BW_SCAL_ODR
    tempRegValue |= (settings.accel.bandwidth & 0x03);
  }
  I2CwriteByte(_xgAddress, CTRL_REG6_XL, tempRegValue);
  // CTRL_REG7_XL (0x21) (Default value: 0x00)
  // [HR][DCF1][DCF0][0][0][FDS][0][HPIS1]
  // HR - High resolution mode (0: disable, 1: enable)
  // DCF[1:0] - Digital filter cutoff frequency
  // FDS - Filtered data selection
  // HPIS1 - HPF enabled for interrupt function
  tempRegValue = 0;
  if (settings.accel.highResEnable)
  {
    tempRegValue |= (1 << 7); // Set HR bit
    tempRegValue |= (settings.accel.highResBandwidth & 0x3) << 5;
  }
  I2CwriteByte(_xgAddress, CTRL_REG7_XL, tempRegValue);
}

void LSM9DS1::initMag()
{
  uint8_t tempRegValue = 0;

  // CTRL_REG1_M (Default value: 0x10)
  // [TEMP_COMP][OM1][OM0][DO2][DO1][DO0][0][ST]
  // TEMP_COMP - Temperature compensation
  // OM[1:0] - X & Y axes op mode selection
  //  00:low-power, 01:medium performance
  //  10: high performance, 11:ultra-high performance
  // DO[2:0] - Output data rate selection
  // ST - Self-test enable
  if (settings.mag.tempCompensationEnable) tempRegValue |= (1 << 7);
  tempRegValue |= (settings.mag.XYPerformance & 0x3) << 5;
  tempRegValue |= (settings.mag.sampleRate & 0x7) << 2;
  I2CwriteByte(_mAddress, CTRL_REG1_M, tempRegValue);

  // CTRL_REG2_M (Default value 0x00)
  // [0][FS1][FS0][0][REBOOT][SOFT_RST][0][0]
  // FS[1:0] - Full-scale configuration
  // REBOOT - Reboot memory content (0:normal, 1:reboot)
  // SOFT_RST - Reset config and user registers (0:default, 1:reset)
  tempRegValue = 0;
  switch (settings.mag.scale)
  {
    case 8:tempRegValue |= (0x1 << 5);
      break;
    case 12:tempRegValue |= (0x2 << 5);
      break;
    case 16:tempRegValue |= (0x3 << 5);
      break;
      // Otherwise we'll default to 4 gauss (00)
  }
  I2CwriteByte(_mAddress, CTRL_REG2_M, tempRegValue); // +/-4Gauss

  // CTRL_REG3_M (Default value: 0x03)
  // [I2C_DISABLE][0][LP][0][0][SIM][MD1][MD0]
  // I2C_DISABLE - Disable I2C interace (0:enable, 1:disable)
  // LP - Low-power mode cofiguration (1:enable)
  // SIM - SPI mode selection (0:write-only, 1:read/write enable)
  // MD[1:0] - Operating mode
  //  00:continuous conversion, 01:single-conversion,
  //  10,11: Power-down
  tempRegValue = 0;
  if (settings.mag.lowPowerEnable) tempRegValue |= (1 << 5);
  tempRegValue |= (settings.mag.operatingMode & 0x3);
  I2CwriteByte(_mAddress, CTRL_REG3_M, tempRegValue);// Continuous conversion mode

  // CTRL_REG4_M (Default value: 0x00)
  // [0][0][0][0][OMZ1][OMZ0][BLE][0]
  // OMZ[1:0] - Z-axis operative mode selection
  //  00:low-power mode, 01:medium performance
  //  10:high performance, 10:ultra-high performance
  // BLE - Big/little endian data
  tempRegValue = 0;
  tempRegValue = (settings.mag.ZPerformance & 0x3) << 2;
  I2CwriteByte(_mAddress, CTRL_REG4_M, tempRegValue);

  // CTRL_REG5_M (Default value: 0x00)
  // [0][BDU][0][0][0][0][0][0]
  // BDU - Block data update for magnetic data
  //  0:continuous, 1:not updated until MSB/LSB are read
  tempRegValue = 0;
  I2CwriteByte(_mAddress, CTRL_REG5_M, tempRegValue);
}
void LSM9DS1::readAccel()
{
  uint8_t temp[6]; // We'll read six bytes from the accelerometer into temp
  xgReadBytes(OUT_X_L_XL, temp, 6); // Read 6 bytes, beginning at OUT_X_L_XL
  ax = (temp[1] << 8) | temp[0]; // Store x-axis values into ax
  ay = (temp[3] << 8) | temp[2]; // Store y-axis values into ay
  az = (temp[5] << 8) | temp[4]; // Store z-axis values into az
  if (_autoCalc)
  {
    ax -= aBiasRaw[X_AXIS];
    ay -= aBiasRaw[Y_AXIS];
    az -= aBiasRaw[Z_AXIS];
  }
}

int16_t LSM9DS1::readAccel(lsm9ds1_axis axis)
{
  uint8_t temp[2];
  int16_t value;
  xgReadBytes(OUT_X_L_XL + (2 * axis), temp, 2);
  value = (temp[1] << 8) | temp[0];

  if (_autoCalc)
    value -= aBiasRaw[axis];

  return value;
}

void LSM9DS1::readMag()
{
  uint8_t temp[6]; // We'll read six bytes from the mag into temp
  mReadBytes(OUT_X_L_M, temp, 6); // Read 6 bytes, beginning at OUT_X_L_M
  mx = (temp[1] << 8) | temp[0]; // Store x-axis values into mx
  my = (temp[3] << 8) | temp[2]; // Store y-axis values into my
  mz = (temp[5] << 8) | temp[4]; // Store z-axis values into mz
}

int16_t LSM9DS1::readMag(lsm9ds1_axis axis)
{
  uint8_t temp[2];
  mReadBytes(OUT_X_L_M + (2 * axis), temp, 2);
  return (temp[1] << 8) | temp[0];
}

void LSM9DS1::readTemp()
{
  uint8_t temp[2]; // We'll read two bytes from the temperature sensor into temp
  xgReadBytes(OUT_TEMP_L, temp, 2); // Read 2 bytes, beginning at OUT_TEMP_L
  temperature = ((int16_t) temp[1] << 8) | temp[0];
}

void LSM9DS1::readGyro()
{
  uint8_t temp[6]; // We'll read six bytes from the gyro into temp
  xgReadBytes(OUT_X_L_G, temp, 6); // Read 6 bytes, beginning at OUT_X_L_G
  gx = (temp[1] << 8) | temp[0]; // Store x-axis values into gx
  gy = (temp[3] << 8) | temp[2]; // Store y-axis values into gy
  gz = (temp[5] << 8) | temp[4]; // Store z-axis values into gz
  if (_autoCalc)
  {
    gx -= gBiasRaw[X_AXIS];
    gy -= gBiasRaw[Y_AXIS];
    gz -= gBiasRaw[Z_AXIS];
  }
}

int16_t LSM9DS1::readGyro(lsm9ds1_axis axis)
{
  uint8_t temp[2];
  int16_t value;

  xgReadBytes(OUT_X_L_G + (2 * axis), temp, 2);

  value = (temp[1] << 8) | temp[0];

  if (_autoCalc)
    value -= gBiasRaw[axis];

  return value;
}

float LSM9DS1::calcGyro(int16_t gyro)
{
  // Return the gyro raw reading times our pre-calculated DPS / (ADC tick):
  return gRes * gyro;
}

float LSM9DS1::calcAccel(int16_t accel)
{
  // Return the accel raw reading times our pre-calculated g's / (ADC tick):
  return aRes * accel;
}

float LSM9DS1::calcMag(int16_t mag)
{
  // Return the mag raw reading times our pre-calculated Gs / (ADC tick):
  return mRes * mag;
}

void LSM9DS1::xgWriteByte(uint8_t subAddress, uint8_t data)
{
  // Whether we're using I2C or SPI, write a byte using the
  // gyro-specific I2C address or SPI CS pin.
  if (settings.device.commInterface == IMU_MODE_I2C)
  {
    I2CwriteByte(_xgAddress, subAddress, data);
  }
//  else if (settings.device.commInterface == IMU_MODE_SPI) {
//    SPIwriteByte(_xgAddress, subAddress, data);
//  }
}

void LSM9DS1::mWriteByte(uint8_t subAddress, uint8_t data)
{
  // Whether we're using I2C or SPI, write a byte using the
  // accelerometer-specific I2C address or SPI CS pin.
  if (settings.device.commInterface == IMU_MODE_I2C)
  {
    return I2CwriteByte(_mAddress, subAddress, data);
  }
//  else if (settings.device.commInterface == IMU_MODE_SPI)
//    return SPIwriteByte(_mAddress, subAddress, data);
}

uint8_t LSM9DS1::xgReadByte(uint8_t subAddress)
{
  // Whether we're using I2C or SPI, read a byte using the
  // gyro-specific I2C address or SPI CS pin.
  if (settings.device.commInterface == IMU_MODE_I2C)
    return I2CreadByte(_xgAddress, subAddress);
//  else if (settings.device.commInterface == IMU_MODE_SPI)
//    return SPIreadByte(_xgAddress, subAddress);
}

void LSM9DS1::xgReadBytes(uint8_t subAddress, uint8_t *dest, uint8_t count)
{
  // Whether we're using I2C or SPI, read multiple bytes using the
  // gyro-specific I2C address or SPI CS pin.
  if (settings.device.commInterface == IMU_MODE_I2C)
  {
    I2CreadBytes(_xgAddress, subAddress, dest, count);
  }
//  else if (settings.device.commInterface == IMU_MODE_SPI) {
//    SPIreadBytes(_xgAddress, subAddress, dest, count);
//  }
}

uint8_t LSM9DS1::mReadByte(uint8_t subAddress)
{
  // Whether we're using I2C or SPI, read a byte using the
  // accelerometer-specific I2C address or SPI CS pin.
  if (settings.device.commInterface == IMU_MODE_I2C)
    return I2CreadByte(_mAddress, subAddress);
//  else if (settings.device.commInterface == IMU_MODE_SPI)
//    return SPIreadByte(_mAddress, subAddress);
}

void LSM9DS1::mReadBytes(uint8_t subAddress, uint8_t *dest, uint8_t count)
{
  // Whether we're using I2C or SPI, read multiple bytes using the
  // accelerometer-specific I2C address or SPI CS pin.
  if (settings.device.commInterface == IMU_MODE_I2C)
    I2CreadBytes(_mAddress, subAddress, dest, count);
//  else if (settings.device.commInterface == IMU_MODE_SPI)
//    SPIreadBytes(_mAddress, subAddress, dest, count);
}

// Wire.h read and write protocols
void LSM9DS1::I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  /*
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
  */
  char temp_data[2] = {subAddress, data};

  _i2c.lock();
  _i2c.write(address, temp_data, 2);
  _i2c.unlock();
}

uint8_t LSM9DS1::I2CreadByte(uint8_t address, uint8_t subAddress)
{
  /*
  int timeout = LSM9DS1_COMMUNICATION_TIMEOUT;
  uint8_t data; // `data` will store the register data
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(true);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address
  while ((Wire.available() < 1) && (timeout-- > 0))
      delay(1);
  if (timeout <= 0)
      return 255; //! Bad! 255 will be misinterpreted as a good value.
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
  */
  char data;
  char temp[2] = {subAddress};

  _i2c.lock();
  _i2c.write(address, temp, 1);
  //i2c.write(address & 0xFE);
  temp[1] = 0x00;
  _i2c.write(address, temp, 1);
  //i2c.write( address | 0x01);
  int a = _i2c.read(address, &data, 1);
  _i2c.unlock();
  return data;
}

uint8_t LSM9DS1::I2CreadBytes(uint8_t address, uint8_t subAddress, uint8_t *dest, uint8_t count)
{
  /*
  int timeout = LSM9DS1_COMMUNICATION_TIMEOUT;
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  // Next send the register to be read. OR with 0x80 to indicate multi-read.
  Wire.write(subAddress | 0x80);     // Put slave register address in Tx buffer
  Wire.endTransmission(true);             // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(address, count);  // Read bytes from slave register address
  while ((Wire.available() < count) && (timeout-- > 0))
      delay(1);
  if (timeout <= 0)
      return -1;
  for (int i=0; i<count;)
  {
      if (Wire.available())
      {
          dest[i++] = Wire.read();
      }
  }
  return count;
  */
  int i;
  char temp_dest[count];
  char temp[1] = {subAddress};

  _i2c.lock();
  _i2c.write(address, temp, 1);
  _i2c.read(address, temp_dest, count);
  _i2c.unlock();

  //i2c doesn't take uint8_ts, but rather chars so do this nasty af conversion
  for (i = 0; i < count; i++)
  {
    dest[i] = temp_dest[i];
  }
  return count;
}

uint8_t LSM9DS1::accelAvailable()
{
  uint8_t status = xgReadByte(STATUS_REG_1);

  return (status & (1 << 0));
}

uint8_t LSM9DS1::gyroAvailable()
{
  uint8_t status = xgReadByte(STATUS_REG_1);

  return ((status & (1 << 1)) >> 1);
}

uint8_t LSM9DS1::tempAvailable()
{
  uint8_t status = xgReadByte(STATUS_REG_1);

  return ((status & (1 << 2)) >> 2);
}

uint8_t LSM9DS1::magAvailable(lsm9ds1_axis axis)
{
  uint8_t status;
  status = mReadByte(STATUS_REG_M);

  return ((status & (1 << axis)) >> axis);
}

void LSM9DS1::calibrate(bool autoCalc)
{
  uint8_t data[6] = {0, 0, 0, 0, 0, 0};
  uint8_t samples = 0;
  int ii;
  int32_t aBiasRawTemp[3] = {0, 0, 0};
  int32_t gBiasRawTemp[3] = {0, 0, 0};

  // Turn on FIFO and set threshold to 32 samples
  enableFIFO(true);
  setFIFO(FIFO_THS, 0x1F);
  while (samples < 0x1F)
  {
    samples = (xgReadByte(FIFO_SRC) & 0x3F); // Read number of stored samples
  }
  for (ii = 0; ii < samples; ii++)
  {   // Read the gyro data stored in the FIFO
    readGyro();
    gBiasRawTemp[0] += gx;
    gBiasRawTemp[1] += gy;
    gBiasRawTemp[2] += gz;
    readAccel();
    aBiasRawTemp[0] += ax;
    aBiasRawTemp[1] += ay;
    aBiasRawTemp[2] += az - (int16_t) (1. / aRes); // Assumes sensor facing up!
  }
  for (ii = 0; ii < 3; ii++)
  {
    gBiasRaw[ii] = gBiasRawTemp[ii] / samples;
    gBias[ii] = calcGyro(gBiasRaw[ii]);
    aBiasRaw[ii] = aBiasRawTemp[ii] / samples;
    aBias[ii] = calcAccel(aBiasRaw[ii]);
  }

  enableFIFO(false);
  setFIFO(FIFO_OFF, 0x00);

  if (autoCalc) _autoCalc = true;
}

void LSM9DS1::calibrateMag(bool loadIn)
{
  int i, j;
  int16_t magMin[3] = {0, 0, 0};
  int16_t magMax[3] = {0, 0, 0}; // The road warrior

  for (i = 0; i < 128; i++)
  {
    while (!magAvailable());
    readMag();
    int16_t magTemp[3] = {0, 0, 0};
    magTemp[0] = mx;
    magTemp[1] = my;
    magTemp[2] = mz;
    for (j = 0; j < 3; j++)
    {
      if (magTemp[j] > magMax[j]) magMax[j] = magTemp[j];
      if (magTemp[j] < magMin[j]) magMin[j] = magTemp[j];
    }
  }
  for (j = 0; j < 3; j++)
  {
    mBiasRaw[j] = (magMax[j] + magMin[j]) / 2;
    mBias[j] = calcMag(mBiasRaw[j]);
    if (loadIn)
      magOffset(j, mBiasRaw[j]);
  }

}

void LSM9DS1::magOffset(uint8_t axis, int16_t offset)
{
  if (axis > 2)
    return;
  uint8_t msb, lsb;
  msb = (offset & 0xFF00) >> 8;
  lsb = offset & 0x00FF;
  mWriteByte(OFFSET_X_REG_L_M + (2 * axis), lsb);
  mWriteByte(OFFSET_X_REG_H_M + (2 * axis), msb);
}

void LSM9DS1::enableFIFO(bool enable)
{
  uint8_t temp = xgReadByte(CTRL_REG9);
  if (enable) temp |= (1 << 1);
  else temp &= ~(1 << 1);
  xgWriteByte(CTRL_REG9, temp);
}

void LSM9DS1::setFIFO(fifoMode_type fifoMode, uint8_t fifoThs)
{
  // Limit threshold - 0x1F (31) is the maximum. If more than that was asked
  // limit it to the maximum.
  uint8_t threshold = fifoThs <= 0x1F ? fifoThs : 0x1F;
  xgWriteByte(FIFO_CTRL, ((fifoMode & 0x7) << 5) | (threshold & 0x1F));
}

void LSM9DS1::update()
{
  if (gyroAvailable())
  {
    readGyro();
  }
  if (accelAvailable())
  {
    readAccel();
  }
  if (magAvailable())
  {
    readMag();
  }
}

float LSM9DS1::getRoll()
{
  return (atan2((float)ay, (float)az) * 180) / M_PI;
}

float LSM9DS1::getPitch()
{
  return (atan2(((-1) * ax), sqrt((float)ay * ay + az * az)) * 180) / M_PI;
}

float LSM9DS1::getHeading()
{
  if (my == 0)
    heading = (mx < 0) ? M_PI : 0;
  else
    heading = atan2((mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch)),
                    (mx * cos(pitch) + mz * sin(pitch)));
  //heading = atan2(sens.mx, sens.my);

  //heading -= DECLINATION * M_PI / 180;

  if (heading > M_PI) heading -= (2 * M_PI);
  else if (heading < (-1 * M_PI)) heading += (2 * M_PI);
  else if (heading < 0) heading += (2 * M_PI);

  heading *= 180.0 / M_PI;

  return heading;
}
