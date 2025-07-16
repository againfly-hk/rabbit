#ifndef BMI088_H
#define BMI088_H

#include <cstdint>

typedef struct {
    int16_t status;
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t temperature;
} BMI088RawData;

typedef struct {
    int16_t status;
    double accel_x, accel_y, accel_z;
    double gyro_x, gyro_y, gyro_z;
    float temperature;
    float time;
} BMI088RealData;

class BMI088 {
public:
    BMI088();
    ~BMI088();

    uint8_t readAccel(void);
    uint8_t readGyro(void);
    uint8_t readTemperature(void);

    BMI088RawData raw_data;
    BMI088RealData real_data;

private:
    int spiHandle;

    uint8_t accelInit(void);
    uint8_t gyroInit(void);
    uint8_t accelSelfTest(void);
    uint8_t gyroSelfTest(void);

    uint8_t writeAccelRegister(uint8_t reg, uint8_t cmd);
    uint8_t readAccelRegister(uint8_t reg);
    uint8_t readAccelMultiRegister(uint8_t reg, uint8_t *bufp, uint8_t len);

    uint8_t writeGyroRegister(uint8_t reg, uint8_t cmd);
    uint8_t readGyroRegister(uint8_t reg);
    uint8_t readGyroMultiRegister(uint8_t reg, uint8_t *bufp, uint8_t len);

    void bmi088CsInit(void);
    void bmi088AccelCsHigh(void);
    void bmi088AccelCsLow(void);
    void bmi088GyroCsHigh(void);
    void bmi088GyroCsLow(void);
    void bmi088SleepMs(unsigned int ms);
    void bmi088SleepUs(unsigned int us);
};

#endif
