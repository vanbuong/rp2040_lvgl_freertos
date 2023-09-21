#include <math.h>
#include "mpu6050.h"
#include "FreeRTOS.h"
#include "task.h"

// Private variable
Vector ra, rg; // Raw vectors
Vector na, ng; // Normalized vectors
Vector tg, dg; // Threshold and Delta for Gyro
Vector th;     // Threshold
Activities a;   // Activities
float dpsPerDigit, rangePerDigit;
float actualThreshold;
bool useCalibrate;
int mpuAddress;

static uint8_t mpu6050_fastRegister8(uint8_t reg);
static uint8_t mpu6050_readRegister8(uint8_t reg);
static void mpu6050_writeRegister8(uint8_t reg, uint8_t value);
static int16_t mpu6050_readRegister16(uint8_t reg);
static void mpu6050_writeRegister16(uint8_t reg, int16_t value);
static bool mpu6050_readRegisterBit(uint8_t reg, uint8_t pos);
static void mpu6050_writeRegisterBit(uint8_t reg, uint8_t pos, bool state);

void mpu6050_hal_init(void)
{
    i2c_init(MPU6050_I2C_CHAN, MPU6050_I2C_SPEED);
    gpio_set_function(MPU6050_I2C_SCL, GPIO_FUNC_I2C);
    gpio_set_function(MPU6050_I2C_SDA, GPIO_FUNC_I2C);
}

bool mpu6050_begin(mpu6050_dps_t scale, mpu6050_range_t range, int mpua)
{
    // Set Address
    mpuAddress = mpua;

    mpu6050_hal_init();

    // Reset calibrate values
    dg.XAxis = 0;
    dg.YAxis = 0;
    dg.ZAxis = 0;
    useCalibrate = false;

    // Reset threshold values
    tg.XAxis = 0;
    tg.YAxis = 0;
    tg.ZAxis = 0;
    actualThreshold = 0;

    // Check MPU6050 Who Am I Register
    if (mpu6050_fastRegister8(MPU6050_REG_WHO_AM_I) != 0x68)
    {
	    return false;
    }

    // Set Clock Source
    mpu6050_setClockSource(MPU6050_CLOCK_PLL_XGYRO);

    // Set Scale & Range
    mpu6050_setScale(scale);
    mpu6050_setRange(range);

    // Disable Sleep Mode
    mpu6050_setSleepEnabled(false);

    return true;
}

void mpu6050_setScale(mpu6050_dps_t scale)
{
    uint8_t value;

    switch (scale)
    {
	case MPU6050_SCALE_250DPS:
	    dpsPerDigit = .007633f;
	    break;
	case MPU6050_SCALE_500DPS:
	    dpsPerDigit = .015267f;
	    break;
	case MPU6050_SCALE_1000DPS:
	    dpsPerDigit = .030487f;
	    break;
	case MPU6050_SCALE_2000DPS:
	    dpsPerDigit = .060975f;
	    break;
	default:
	    break;
    }

    value = mpu6050_readRegister8(MPU6050_REG_GYRO_CONFIG);
    value &= 0b11100111;
    value |= (scale << 3);
    mpu6050_writeRegister8(MPU6050_REG_GYRO_CONFIG, value);
}

mpu6050_dps_t mpu6050_getScale(void)
{
    uint8_t value;
    value = mpu6050_readRegister8(MPU6050_REG_GYRO_CONFIG);
    value &= 0b00011000;
    value >>= 3;
    return (mpu6050_dps_t)value;
}

void mpu6050_setRange(mpu6050_range_t range)
{
    uint8_t value;

    switch (range)
    {
	case MPU6050_RANGE_2G:
	    rangePerDigit = .000061f;
	    break;
	case MPU6050_RANGE_4G:
	    rangePerDigit = .000122f;
	    break;
	case MPU6050_RANGE_8G:
	    rangePerDigit = .000244f;
	    break;
	case MPU6050_RANGE_16G:
	    rangePerDigit = .0004882f;
	    break;
	default:
	    break;
    }

    value = mpu6050_readRegister8(MPU6050_REG_ACCEL_CONFIG);
    value &= 0b11100111;
    value |= (range << 3);
    mpu6050_writeRegister8(MPU6050_REG_ACCEL_CONFIG, value);
}

mpu6050_range_t mpu6050_getRange(void)
{
    uint8_t value;
    value = mpu6050_readRegister8(MPU6050_REG_ACCEL_CONFIG);
    value &= 0b00011000;
    value >>= 3;
    return (mpu6050_range_t)value;
}

void mpu6050_setDHPFMode(mpu6050_dhpf_t dhpf)
{
    uint8_t value;
    value = mpu6050_readRegister8(MPU6050_REG_ACCEL_CONFIG);
    value &= 0b11111000;
    value |= dhpf;
    mpu6050_writeRegister8(MPU6050_REG_ACCEL_CONFIG, value);
}

void mpu6050_setDLPFMode(mpu6050_dlpf_t dlpf)
{
    uint8_t value;
    value = mpu6050_readRegister8(MPU6050_REG_CONFIG);
    value &= 0b11111000;
    value |= dlpf;
    mpu6050_writeRegister8(MPU6050_REG_CONFIG, value);
}

void mpu6050_setClockSource(mpu6050_clockSource_t source)
{
    uint8_t value;
    value = mpu6050_readRegister8(MPU6050_REG_PWR_MGMT_1);
    value &= 0b11111000;
    value |= source;
    mpu6050_writeRegister8(MPU6050_REG_PWR_MGMT_1, value);
}

mpu6050_clockSource_t mpu6050_getClockSource(void)
{
    uint8_t value;
    value = mpu6050_readRegister8(MPU6050_REG_PWR_MGMT_1);
    value &= 0b00000111;
    return (mpu6050_clockSource_t)value;
}

bool mpu6050_getSleepEnabled(void)
{
    return mpu6050_readRegisterBit(MPU6050_REG_PWR_MGMT_1, 6);
}

void mpu6050_setSleepEnabled(bool state)
{
    mpu6050_writeRegisterBit(MPU6050_REG_PWR_MGMT_1, 6, state);
}

bool mpu6050_getIntZeroMotionEnabled(void)
{
    return mpu6050_readRegisterBit(MPU6050_REG_INT_ENABLE, 5);
}

void mpu6050_setIntZeroMotionEnabled(bool state)
{
    mpu6050_writeRegisterBit(MPU6050_REG_INT_ENABLE, 5, state);
}

bool mpu6050_getIntMotionEnabled(void)
{
    return mpu6050_readRegisterBit(MPU6050_REG_INT_ENABLE, 6);
}

void mpu6050_setIntMotionEnabled(bool state)
{
    mpu6050_writeRegisterBit(MPU6050_REG_INT_ENABLE, 6, state);
}

bool mpu6050_getIntFreeFallEnabled(void)
{
    return mpu6050_readRegisterBit(MPU6050_REG_INT_ENABLE, 7);
}

void mpu6050_setIntFreeFallEnabled(bool state)
{
    mpu6050_writeRegisterBit(MPU6050_REG_INT_ENABLE, 7, state);
}

uint8_t mpu6050_getMotionDetectionThreshold(void)
{
    return mpu6050_readRegister8(MPU6050_REG_MOT_THRESHOLD);
}

void mpu6050_setMotionDetectionThreshold(uint8_t threshold)
{
    mpu6050_writeRegister8(MPU6050_REG_MOT_THRESHOLD, threshold);
}

uint8_t mpu6050_getMotionDetectionDuration(void)
{
    return mpu6050_readRegister8(MPU6050_REG_MOT_DURATION);
}

void mpu6050_setMotionDetectionDuration(uint8_t duration)
{
    mpu6050_writeRegister8(MPU6050_REG_MOT_DURATION, duration);
}

uint8_t mpu6050_getZeroMotionDetectionThreshold(void)
{
    return mpu6050_readRegister8(MPU6050_REG_ZMOT_THRESHOLD);
}

void mpu6050_setZeroMotionDetectionThreshold(uint8_t threshold)
{
    mpu6050_writeRegister8(MPU6050_REG_ZMOT_THRESHOLD, threshold);
}

uint8_t mpu6050_getZeroMotionDetectionDuration(void)
{
    return mpu6050_readRegister8(MPU6050_REG_ZMOT_DURATION);
}

void mpu6050_setZeroMotionDetectionDuration(uint8_t duration)
{
    mpu6050_writeRegister8(MPU6050_REG_ZMOT_DURATION, duration);
}

uint8_t mpu6050_getFreeFallDetectionThreshold(void)
{
    return mpu6050_readRegister8(MPU6050_REG_FF_THRESHOLD);
}

void mpu6050_setFreeFallDetectionThreshold(uint8_t threshold)
{
    mpu6050_writeRegister8(MPU6050_REG_FF_THRESHOLD, threshold);
}

uint8_t mpu6050_getFreeFallDetectionDuration(void)
{
    return mpu6050_readRegister8(MPU6050_REG_FF_DURATION);
}

void mpu6050_setFreeFallDetectionDuration(uint8_t duration)
{
    mpu6050_writeRegister8(MPU6050_REG_FF_DURATION, duration);
}

bool mpu6050_getI2CMasterModeEnabled(void)
{
    return mpu6050_readRegisterBit(MPU6050_REG_USER_CTRL, 5);
}

void mpu6050_setI2CMasterModeEnabled(bool state)
{
    mpu6050_writeRegisterBit(MPU6050_REG_USER_CTRL, 5, state);
}

void mpu6050_setI2CBypassEnabled(bool state)
{
    return mpu6050_writeRegisterBit(MPU6050_REG_INT_PIN_CFG, 1, state);
}

bool mpu6050_getI2CBypassEnabled(void)
{
    return mpu6050_readRegisterBit(MPU6050_REG_INT_PIN_CFG, 1);
}

void mpu6050_setAccelPowerOnDelay(mpu6050_onDelay_t delay)
{
    uint8_t value;
    value = mpu6050_readRegister8(MPU6050_REG_MOT_DETECT_CTRL);
    value &= 0b11001111;
    value |= (delay << 4);
    mpu6050_writeRegister8(MPU6050_REG_MOT_DETECT_CTRL, value);
}

mpu6050_onDelay_t mpu6050_getAccelPowerOnDelay(void)
{
    uint8_t value;
    value = mpu6050_readRegister8(MPU6050_REG_MOT_DETECT_CTRL);
    value &= 0b00110000;
    return (mpu6050_onDelay_t)(value >> 4);
}

uint8_t mpu6050_getIntStatus(void)
{
    return mpu6050_readRegister8(MPU6050_REG_INT_STATUS);
}

Activities mpu6050_readActivites(void)
{
    uint8_t data = mpu6050_readRegister8(MPU6050_REG_INT_STATUS);

    a.isOverflow = ((data >> 4) & 1);
    a.isFreeFall = ((data >> 7) & 1);
    a.isInactivity = ((data >> 5) & 1);
    a.isActivity = ((data >> 6) & 1);
    a.isDataReady = ((data >> 0) & 1);

    data = mpu6050_readRegister8(MPU6050_REG_MOT_DETECT_STATUS);

    a.isNegActivityOnX = ((data >> 7) & 1);
    a.isPosActivityOnX = ((data >> 6) & 1);

    a.isNegActivityOnY = ((data >> 5) & 1);
    a.isPosActivityOnY = ((data >> 4) & 1);

    a.isNegActivityOnZ = ((data >> 3) & 1);
    a.isPosActivityOnZ = ((data >> 2) & 1);

    return a;
}

Vector mpu6050_readRawAccel(void)
{
    uint8_t data[6] = {MPU6050_REG_ACCEL_XOUT_H};
    i2c_write_blocking(MPU6050_I2C_CHAN, MPU6050_ADDRESS, data, 1, true);
    i2c_read_blocking(MPU6050_I2C_CHAN, MPU6050_ADDRESS, data, 6, false);

    ra.XAxis = (int16_t)(data[0] << 8 | data[1]);
    ra.YAxis = (int16_t)(data[2] << 8 | data[3]);
    ra.ZAxis = (int16_t)(data[4] << 8 | data[5]);

    return ra;
}

Vector mpu6050_readNormalizeAccel(void)
{
    mpu6050_readRawAccel();

    na.XAxis = ra.XAxis * rangePerDigit * 9.80665f;
    na.YAxis = ra.YAxis * rangePerDigit * 9.80665f;
    na.ZAxis = ra.ZAxis * rangePerDigit * 9.80665f;

    return na;
}

Vector mpu6050_readScaledAccel(void)
{
    mpu6050_readRawAccel();

    na.XAxis = ra.XAxis * rangePerDigit;
    na.YAxis = ra.YAxis * rangePerDigit;
    na.ZAxis = ra.ZAxis * rangePerDigit;

    return na;
}


Vector mpu6050_readRawGyro(void)
{
    uint8_t data[6] = {MPU6050_REG_GYRO_XOUT_H};
    i2c_write_blocking(MPU6050_I2C_CHAN, MPU6050_ADDRESS, data, 1, true);
    i2c_read_blocking(MPU6050_I2C_CHAN, MPU6050_ADDRESS, data, 6, false);

    rg.XAxis = (int16_t)(data[0] << 8 | data[1]);
    rg.YAxis = (int16_t)(data[2] << 8 | data[3]);
    rg.ZAxis = (int16_t)(data[4] << 8 | data[5]);

    return rg;
}

Vector mpu6050_readNormalizeGyro(void)
{
    mpu6050_readRawGyro();

    if (useCalibrate) {
        ng.XAxis = (rg.XAxis - dg.XAxis) * dpsPerDigit;
        ng.YAxis = (rg.YAxis - dg.YAxis) * dpsPerDigit;
        ng.ZAxis = (rg.ZAxis - dg.ZAxis) * dpsPerDigit;
    } else {
        ng.XAxis = rg.XAxis * dpsPerDigit;
        ng.YAxis = rg.YAxis * dpsPerDigit;
        ng.ZAxis = rg.ZAxis * dpsPerDigit;
    }

    if (actualThreshold) {
        if (fabsf(ng.XAxis) < tg.XAxis) ng.XAxis = 0;
        if (fabsf(ng.YAxis) < tg.YAxis) ng.YAxis = 0;
        if (fabsf(ng.ZAxis) < tg.ZAxis) ng.ZAxis = 0;
    }

    return ng;
}

float mpu6050_readTemperature(void)
{
    int16_t T;
    T = mpu6050_readRegister16(MPU6050_REG_TEMP_OUT_H);
    return (float)T/340 + 36.53;
}

int16_t mpu6050_getGyroOffsetX(void)
{
    return mpu6050_readRegister16(MPU6050_REG_GYRO_XOFFS_H);
}

int16_t mpu6050_getGyroOffsetY(void)
{
    return mpu6050_readRegister16(MPU6050_REG_GYRO_YOFFS_H);
}

int16_t mpu6050_getGyroOffsetZ(void)
{
    return mpu6050_readRegister16(MPU6050_REG_GYRO_ZOFFS_H);
}

void mpu6050_setGyroOffsetX(int16_t offset)
{
    mpu6050_writeRegister16(MPU6050_REG_GYRO_XOFFS_H, offset);
}

void mpu6050_setGyroOffsetY(int16_t offset)
{
    mpu6050_writeRegister16(MPU6050_REG_GYRO_YOFFS_H, offset);
}

void mpu6050_setGyroOffsetZ(int16_t offset)
{
    mpu6050_writeRegister16(MPU6050_REG_GYRO_ZOFFS_H, offset);
}

int16_t mpu6050_getAccelOffsetX(void)
{
    return mpu6050_readRegister16(MPU6050_REG_ACCEL_XOFFS_H);
}

int16_t mpu6050_getAccelOffsetY(void)
{
    return mpu6050_readRegister16(MPU6050_REG_ACCEL_YOFFS_H);
}

int16_t mpu6050_getAccelOffsetZ(void)
{
    return mpu6050_readRegister16(MPU6050_REG_ACCEL_ZOFFS_H);
}

void mpu6050_setAccelOffsetX(int16_t offset)
{
    mpu6050_writeRegister16(MPU6050_REG_ACCEL_XOFFS_H, offset);
}

void mpu6050_setAccelOffsetY(int16_t offset)
{
    mpu6050_writeRegister16(MPU6050_REG_ACCEL_YOFFS_H, offset);
}

void mpu6050_setAccelOffsetZ(int16_t offset)
{
    mpu6050_writeRegister16(MPU6050_REG_ACCEL_ZOFFS_H, offset);
}

// Calibrate algorithm
void mpu6050_calibrateGyro(uint8_t samples)
{
    // Set calibrate
    useCalibrate = true;

    // Reset values
    float sumX = 0;
    float sumY = 0;
    float sumZ = 0;
    float sigmaX = 0;
    float sigmaY = 0;
    float sigmaZ = 0;

    // Read n-samples
    for (uint8_t i = 0; i < samples; ++i)
    {
        mpu6050_readRawGyro();
        sumX += rg.XAxis;
        sumY += rg.YAxis;
        sumZ += rg.ZAxis;

        sigmaX += rg.XAxis * rg.XAxis;
        sigmaY += rg.YAxis * rg.YAxis;
        sigmaZ += rg.ZAxis * rg.ZAxis;

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    // Calculate delta vectors
    dg.XAxis = sumX / samples;
    dg.YAxis = sumY / samples;
    dg.ZAxis = sumZ / samples;

    // Calculate threshold vectors
    th.XAxis = sqrt((sigmaX / samples) - (dg.XAxis * dg.XAxis));
    th.YAxis = sqrt((sigmaY / samples) - (dg.YAxis * dg.YAxis));
    th.ZAxis = sqrt((sigmaZ / samples) - (dg.ZAxis * dg.ZAxis));

    // If already set threshold, recalculate threshold vectors
    if (actualThreshold > 0)
    {
	    mpu6050_setThreshold(actualThreshold);
    }
}

// Get current threshold value
uint8_t mpu6050_getThreshold(void)
{
    return actualThreshold;
}

// Set treshold value
void mpu6050_setThreshold(uint8_t multiple)
{
    if (multiple > 0) {
        // If not calibrated, need calibrate
        if (!useCalibrate) {
            mpu6050_calibrateGyro(50);
        }

        // Calculate threshold vectors
        tg.XAxis = th.XAxis * multiple;
        tg.YAxis = th.YAxis * multiple;
        tg.ZAxis = th.ZAxis * multiple;
    } else {
        // No threshold
        tg.XAxis = 0;
        tg.YAxis = 0;
        tg.ZAxis = 0;
    }

    // Remember old threshold value
    actualThreshold = multiple;
}

// Fast read 8-bit from register
uint8_t mpu6050_fastRegister8(uint8_t reg)
{
    uint8_t value;

    i2c_write_blocking(MPU6050_I2C_CHAN, MPU6050_ADDRESS, &reg, 1, true);
    i2c_read_blocking(MPU6050_I2C_CHAN, MPU6050_ADDRESS, &value, 1, false);

    return value;
}

// Read 8-bit from register
uint8_t mpu6050_readRegister8(uint8_t reg)
{
    uint8_t value;

    i2c_write_blocking(MPU6050_I2C_CHAN, MPU6050_ADDRESS, &reg, 1, true);
    i2c_read_blocking(MPU6050_I2C_CHAN, MPU6050_ADDRESS, &value, 1, false);

    return value;
}

// Write 8-bit to register
void mpu6050_writeRegister8(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    i2c_write_blocking(MPU6050_I2C_CHAN, MPU6050_ADDRESS, data, 2, false);
}

int16_t mpu6050_readRegister16(uint8_t reg)
{
    int16_t value;
    uint8_t data[2];
    i2c_write_blocking(MPU6050_I2C_CHAN, MPU6050_ADDRESS, &reg, 1, true);
    i2c_read_blocking(MPU6050_I2C_CHAN, MPU6050_ADDRESS, data, 2, false);

    value = data[0] << 8 | data[1];

    return value;
}

void mpu6050_writeRegister16(uint8_t reg, int16_t value)
{
    uint8_t data[3] = {reg, (value >> 8) & 0xff, value & 0xff};
    i2c_write_blocking(MPU6050_I2C_CHAN, MPU6050_ADDRESS, data, 2, false);
}

// Read register bit
bool mpu6050_readRegisterBit(uint8_t reg, uint8_t pos)
{
    uint8_t value;
    value = mpu6050_readRegister8(reg);
    return ((value >> pos) & 1);
}

// Write register bit
void mpu6050_writeRegisterBit(uint8_t reg, uint8_t pos, bool state)
{
    uint8_t value;
    value = mpu6050_readRegister8(reg);

    if (state) {
        value |= (1 << pos);
    } else {
        value &= ~(1 << pos);
    }

    mpu6050_writeRegister8(reg, value);
}
