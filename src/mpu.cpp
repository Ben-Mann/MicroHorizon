/**
 * MPU6050 IMU setup & read on the Arduino Nano / ATmega328P
 */

#include "mpu.h"
#include <Wire.h>

#define I2CMPU 0x68
#define MPU_ACCEL 0x3B
#define MPU_TEMP 0x41
#define MPU_PWR_MGMT_1 0x6B
#define ACCEL_CONFIG 0x1C
#define AFS_SEL_16G 0x18
#define LSB_PER_G_16G 2048.0f
Vector mpuAccel = Vector();
float mpuTemp;

void mpuSetup() {
    Wire.begin();
    Wire.beginTransmission(I2CMPU);
    Wire.write(MPU_PWR_MGMT_1);
    Wire.write(0x00); // Clear sleep bit; the MPU6050 starts in sleep mode.
    Wire.endTransmission(true);
    Wire.beginTransmission(I2CMPU);
    Wire.write(ACCEL_CONFIG);
    Wire.write(AFS_SEL_16G);
    Wire.endTransmission(false);
}

void mpuGet() {
    Wire.beginTransmission(I2CMPU);
    Wire.write(MPU_ACCEL);
    Wire.endTransmission(false);
    Wire.requestFrom(I2CMPU, 6, true);
    mpuAccel.x = (Wire.read() << 8 | Wire.read()) / LSB_PER_G_16G;
    mpuAccel.y = (Wire.read() << 8 | Wire.read()) / LSB_PER_G_16G;
    mpuAccel.z = (Wire.read() << 8 | Wire.read()) / LSB_PER_G_16G;
    Wire.beginTransmission(I2CMPU);
    Wire.write(MPU_TEMP);
    Wire.endTransmission(false);
    Wire.requestFrom(I2CMPU, 2, true);
    // Formula from datasheet
    mpuTemp = (Wire.read() << 8 | Wire.read()) / 340.0f + 36.53f;
}