#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define MPU6050_I2C_CHAN    i2c1
#define MPU6050_I2C_SDA     18
#define MPU6050_I2C_SCL     19
#define MPU6050_I2C_SPEED   100 * 1000

#define MPU6050_ADDRESS             (0x68) // 0x69 when AD0 pin to Vcc

#define MPU6050_REG_ACCEL_XOFFS_H     (0x06)
#define MPU6050_REG_ACCEL_XOFFS_L     (0x07)
#define MPU6050_REG_ACCEL_YOFFS_H     (0x08)
#define MPU6050_REG_ACCEL_YOFFS_L     (0x09)
#define MPU6050_REG_ACCEL_ZOFFS_H     (0x0A)
#define MPU6050_REG_ACCEL_ZOFFS_L     (0x0B)
#define MPU6050_REG_GYRO_XOFFS_H      (0x13)
#define MPU6050_REG_GYRO_XOFFS_L      (0x14)
#define MPU6050_REG_GYRO_YOFFS_H      (0x15)
#define MPU6050_REG_GYRO_YOFFS_L      (0x16)
#define MPU6050_REG_GYRO_ZOFFS_H      (0x17)
#define MPU6050_REG_GYRO_ZOFFS_L      (0x18)
#define MPU6050_REG_CONFIG            (0x1A)
#define MPU6050_REG_GYRO_CONFIG       (0x1B) // Gyroscope Configuration
#define MPU6050_REG_ACCEL_CONFIG      (0x1C) // Accelerometer Configuration
#define MPU6050_REG_FF_THRESHOLD      (0x1D)
#define MPU6050_REG_FF_DURATION       (0x1E)
#define MPU6050_REG_MOT_THRESHOLD     (0x1F)
#define MPU6050_REG_MOT_DURATION      (0x20)
#define MPU6050_REG_ZMOT_THRESHOLD    (0x21)
#define MPU6050_REG_ZMOT_DURATION     (0x22)
#define MPU6050_REG_INT_PIN_CFG       (0x37) // INT Pin. Bypass Enable Configuration
#define MPU6050_REG_INT_ENABLE        (0x38) // INT Enable
#define MPU6050_REG_INT_STATUS        (0x3A)
#define MPU6050_REG_ACCEL_XOUT_H      (0x3B)
#define MPU6050_REG_ACCEL_XOUT_L      (0x3C)
#define MPU6050_REG_ACCEL_YOUT_H      (0x3D)
#define MPU6050_REG_ACCEL_YOUT_L      (0x3E)
#define MPU6050_REG_ACCEL_ZOUT_H      (0x3F)
#define MPU6050_REG_ACCEL_ZOUT_L      (0x40)
#define MPU6050_REG_TEMP_OUT_H        (0x41)
#define MPU6050_REG_TEMP_OUT_L        (0x42)
#define MPU6050_REG_GYRO_XOUT_H       (0x43)
#define MPU6050_REG_GYRO_XOUT_L       (0x44)
#define MPU6050_REG_GYRO_YOUT_H       (0x45)
#define MPU6050_REG_GYRO_YOUT_L       (0x46)
#define MPU6050_REG_GYRO_ZOUT_H       (0x47)
#define MPU6050_REG_GYRO_ZOUT_L       (0x48)
#define MPU6050_REG_MOT_DETECT_STATUS (0x61)
#define MPU6050_REG_MOT_DETECT_CTRL   (0x69)
#define MPU6050_REG_USER_CTRL         (0x6A) // User Control
#define MPU6050_REG_PWR_MGMT_1        (0x6B) // Power Management 1
#define MPU6050_REG_WHO_AM_I          (0x75) // Who Am I

#ifndef VECTOR_STRUCT_H
#define VECTOR_STRUCT_H
typedef struct Vector_t
{
    float XAxis;
    float YAxis;
    float ZAxis;
} Vector;
#endif

typedef struct Activities_t
{
    bool isOverflow;
    bool isFreeFall;
    bool isInactivity;
    bool isActivity;
    bool isPosActivityOnX;
    bool isPosActivityOnY;
    bool isPosActivityOnZ;
    bool isNegActivityOnX;
    bool isNegActivityOnY;
    bool isNegActivityOnZ;
    bool isDataReady;
} Activities;

typedef enum
{
    MPU6050_CLOCK_KEEP_RESET      = 0b111,
    MPU6050_CLOCK_EXTERNAL_19MHZ  = 0b101,
    MPU6050_CLOCK_EXTERNAL_32KHZ  = 0b100,
    MPU6050_CLOCK_PLL_ZGYRO       = 0b011,
    MPU6050_CLOCK_PLL_YGYRO       = 0b010,
    MPU6050_CLOCK_PLL_XGYRO       = 0b001,
    MPU6050_CLOCK_INTERNAL_8MHZ   = 0b000
} mpu6050_clockSource_t;

typedef enum
{
    MPU6050_SCALE_2000DPS         = 0b11,
    MPU6050_SCALE_1000DPS         = 0b10,
    MPU6050_SCALE_500DPS          = 0b01,
    MPU6050_SCALE_250DPS          = 0b00
} mpu6050_dps_t;

typedef enum
{
    MPU6050_RANGE_16G             = 0b11,
    MPU6050_RANGE_8G              = 0b10,
    MPU6050_RANGE_4G              = 0b01,
    MPU6050_RANGE_2G              = 0b00,
} mpu6050_range_t;

typedef enum
{
    MPU6050_DELAY_3MS             = 0b11,
    MPU6050_DELAY_2MS             = 0b10,
    MPU6050_DELAY_1MS             = 0b01,
    MPU6050_NO_DELAY              = 0b00,
} mpu6050_onDelay_t;

typedef enum
{
    MPU6050_DHPF_HOLD             = 0b111,
    MPU6050_DHPF_0_63HZ           = 0b100,
    MPU6050_DHPF_1_25HZ           = 0b011,
    MPU6050_DHPF_2_5HZ            = 0b010,
    MPU6050_DHPF_5HZ              = 0b001,
    MPU6050_DHPF_RESET            = 0b000,
} mpu6050_dhpf_t;

typedef enum
{
    MPU6050_DLPF_6                = 0b110,
    MPU6050_DLPF_5                = 0b101,
    MPU6050_DLPF_4                = 0b100,
    MPU6050_DLPF_3                = 0b011,
    MPU6050_DLPF_2                = 0b010,
    MPU6050_DLPF_1                = 0b001,
    MPU6050_DLPF_0                = 0b000,
} mpu6050_dlpf_t;

#ifdef __cplusplus
extern "C"
{
#endif

bool mpu6050_begin(mpu6050_dps_t scale, mpu6050_range_t range, int mpua);

void mpu6050_setClockSource(mpu6050_clockSource_t source);
void mpu6050_setScale(mpu6050_dps_t scale);
void mpu6050_setRange(mpu6050_range_t range);
mpu6050_clockSource_t mpu6050_getClockSource(void);
mpu6050_dps_t mpu6050_getScale(void);
mpu6050_range_t mpu6050_getRange(void);
void mpu6050_setDHPFMode(mpu6050_dhpf_t dhpf);
void mpu6050_setDLPFMode(mpu6050_dlpf_t dlpf);
mpu6050_onDelay_t mpu6050_getAccelPowerOnDelay();
void mpu6050_setAccelPowerOnDelay(mpu6050_onDelay_t delay);

uint8_t mpu6050_getIntStatus(void);

bool mpu6050_getIntZeroMotionEnabled(void);
void mpu6050_setIntZeroMotionEnabled(bool state);
bool mpu6050_getIntMotionEnabled(void);
void mpu6050_setIntMotionEnabled(bool state);
bool mpu6050_getIntFreeFallEnabled(void);
void mpu6050_setIntFreeFallEnabled(bool state);

uint8_t mpu6050_getMotionDetectionThreshold(void);
void mpu6050_setMotionDetectionThreshold(uint8_t threshold);
uint8_t mpu6050_getMotionDetectionDuration(void);
void mpu6050_setMotionDetectionDuration(uint8_t duration);

uint8_t mpu6050_getZeroMotionDetectionThreshold(void);
void mpu6050_setZeroMotionDetectionThreshold(uint8_t threshold);
uint8_t mpu6050_getZeroMotionDetectionDuration(void);
void mpu6050_setZeroMotionDetectionDuration(uint8_t duration);

uint8_t mpu6050_getFreeFallDetectionThreshold(void);
void mpu6050_setFreeFallDetectionThreshold(uint8_t threshold);
uint8_t mpu6050_getFreeFallDetectionDuration(void);
void mpu6050_setFreeFallDetectionDuration(uint8_t duration);

bool mpu6050_getSleepEnabled(void);
void mpu6050_setSleepEnabled(bool state);
bool mpu6050_getI2CMasterModeEnabled(void);
void mpu6050_setI2CMasterModeEnabled(bool state);
bool mpu6050_getI2CBypassEnabled(void);
void mpu6050_setI2CBypassEnabled(bool state);

float mpu6050_readTemperature(void);
Activities mpu6050_readActivites(void);

int16_t mpu6050_getGyroOffsetX(void);
void mpu6050_setGyroOffsetX(int16_t offset);
int16_t mpu6050_getGyroOffsetY(void);
void mpu6050_setGyroOffsetY(int16_t offset);
int16_t mpu6050_getGyroOffsetZ(void);
void mpu6050_setGyroOffsetZ(int16_t offset);

int16_t mpu6050_getAccelOffsetX(void);
void mpu6050_setAccelOffsetX(int16_t offset);
int16_t mpu6050_getAccelOffsetY(void);
void mpu6050_setAccelOffsetY(int16_t offset);
int16_t mpu6050_getAccelOffsetZ(void);
void mpu6050_setAccelOffsetZ(int16_t offset);

void mpu6050_calibrateGyro(uint8_t samples);
void mpu6050_setThreshold(uint8_t multiple);
uint8_t mpu6050_getThreshold(void);

Vector mpu6050_readRawGyro(void);
Vector mpu6050_readNormalizeGyro(void);

Vector mpu6050_readRawAccel(void);
Vector mpu6050_readNormalizeAccel(void);
Vector mpu6050_readScaledAccel(void);

#ifdef __cplusplus
}
#endif

#endif
