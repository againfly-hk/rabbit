#include "bmi088.h"
#include "bmi088def.h"
#include "bmi088reg.h"

#include <pigpio.h>
#include <stdexcept>
#include <iostream>

#include <cstdint>
#include <thread>
#include <chrono>

#define csGyro  7
#define csAccel 8

double bmi088_accel_sen = BMI088_ACCEL_3G_SEN;
double bmi088_gyro_sen  = BMI088_GYRO_500_SEN;

BMI088::BMI088() {
    if (gpioInitialise() < 0) {
        throw std::runtime_error("pigpio initialization failed");
    }
    bmi088CsInit();

    spiHandle = spiOpen(0, 5000000, 0);
    if (spiHandle < 0) {
        throw std::runtime_error("spiOpen failed");
    }
    bmi088SleepMs(10);

    // accel self test
    uint8_t accel_error_code = accelSelfTest();
    if (accel_error_code != BMI088_NO_ERROR) {
        spiClose(spiHandle);
        gpioTerminate();
        if (accel_error_code == BMI088_NO_SENSOR) {
            throw std::runtime_error("BMI088 accel not detected");
        } else if (accel_error_code == BMI088_ACCEL_SELF_TEST_ERROR) {
            throw std::runtime_error("BMI088 accel self-test failed");
        } else if (accel_error_code == BMI088_ACCEL_CONF_ERROR) {
            throw std::runtime_error("BMI088 accel config register set/check failed");
        } else if (accel_error_code == BMI088_ACCEL_PWR_CTRL_ERROR) {
            throw std::runtime_error("BMI088 accel power control register set/check failed");
        } else if (accel_error_code == BMI088_ACCEL_RANGE_ERROR) {
            throw std::runtime_error("BMI088 accel range register set/check failed");
        } else if (accel_error_code == BMI088_ACCEL_PWR_CONF_ERROR) {
            throw std::runtime_error("BMI088 accel power config register set/check failed");
        } else if (accel_error_code == BMI088_ACCEL_SELF_TEST_ERROR) {
            throw std::runtime_error("BMI088 accel value self-test failed");
        }
    }

    // gyro self test
    uint8_t gyro_error_code = gyroSelfTest();
    if (gyro_error_code != BMI088_NO_ERROR) {
        spiClose(spiHandle);
        gpioTerminate();
        if (gyro_error_code == BMI088_NO_SENSOR) {
            throw std::runtime_error("BMI088 gyro not detected");
        } else if (gyro_error_code == BMI088_SELF_TEST_GYRO_ERROR) {
            throw std::runtime_error("BMI088 gyro self-test failed");
        }
    }

    // accel init
    accel_error_code = accelInit();
    if (accel_error_code != BMI088_NO_ERROR) {
        spiClose(spiHandle);
        gpioTerminate();
        if (accel_error_code == BMI088_NO_SENSOR) {
            throw std::runtime_error("BMI088 accel not detected");
        } else if (accel_error_code == BMI088_ACCEL_SELF_TEST_ERROR) {
            throw std::runtime_error("BMI088 accel self-test failed");
        } else if (accel_error_code == BMI088_ACCEL_CONF_ERROR) {
            throw std::runtime_error("BMI088 accel config error");
        } else if (accel_error_code == BMI088_ACCEL_PWR_CTRL_ERROR) {
            throw std::runtime_error("BMI088 accel power control error");
        } else if (accel_error_code == BMI088_ACCEL_RANGE_ERROR) {
            throw std::runtime_error("BMI088 accel range error");
        } else if (accel_error_code == BMI088_ACCEL_PWR_CONF_ERROR) {
            throw std::runtime_error("BMI088 accel power config error");
        } else if (accel_error_code == BMI088_INT1_IO_CTRL_ERROR) {
            throw std::runtime_error("BMI088 accel INT1 IO control error");
        } else if (accel_error_code == BMI088_INT_MAP_DATA_ERROR) {
            throw std::runtime_error("BMI088 accel INT map data error");
        } else {
            throw std::runtime_error("BMI088 accel unknown error");
        }
    }
    std::cout << "BMI088 accel init done" << std::endl;

    // gyro init
    gyro_error_code = gyroInit();
    if (gyro_error_code != BMI088_NO_ERROR) {
        spiClose(spiHandle);
        gpioTerminate();
        if (gyro_error_code == BMI088_NO_SENSOR) {
            throw std::runtime_error("BMI088 gyro not detected");
        } else if (gyro_error_code == BMI088_SELF_TEST_GYRO_ERROR) {
            throw std::runtime_error("BMI088 gyro self-test failed");
        } else if (gyro_error_code == BMI088_GYRO_RANGE_ERROR) {
            throw std::runtime_error("BMI088 gyro range error");
        } else if (gyro_error_code == BMI088_GYRO_BANDWIDTH_ERROR) {
            throw std::runtime_error("BMI088 gyro bandwidth error");
        } else if (gyro_error_code == BMI088_GYRO_LPM1_ERROR) {
            throw std::runtime_error("BMI088 gyro low power mode error");
        } else if (gyro_error_code == BMI088_GYRO_CTRL_ERROR) {
            throw std::runtime_error("BMI088 gyro control error");
        } else if (gyro_error_code == BMI088_GYRO_INT3_INT4_IO_CONFIG_ERROR) {
            throw std::runtime_error("BMI088 gyro INT3/INT4 IO conf error");
        } else if (gyro_error_code == BMI088_GYRO_INT3_INT4_IO_MAP_ERROR) {
            throw std::runtime_error("BMI088 gyro INT3/INT4 IO map error");
        } else {
            throw std::runtime_error("BMI088 gyro unknown error");
        }
    }
    std::cout << "BMI088 gyro init done" << std::endl;
}

BMI088::~BMI088() {
    spiClose(spiHandle);
    gpioTerminate();
}

uint8_t BMI088::accelInit(void) {
    uint8_t res = 0;
    uint8_t write_reg_num = 0;
    uint8_t accel_id = 0;
    
    accel_id = readAccelRegister(BMI088_ACCEL_CHIP_ID);
    bmi088SleepUs(BMI088_WAIT_TIME);
    accel_id = readAccelRegister(BMI088_ACCEL_CHIP_ID);
    bmi088SleepUs(BMI088_WAIT_TIME);
    if (accel_id != BMI088_ACCEL_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    writeAccelRegister(BMI088_ACCEL_SOFTRESET, BMI088_ACCEL_SOFTRESET_VALUE);
    bmi088SleepMs(BMI088_LONG_DELAY_TIME);

    accel_id = readAccelRegister(BMI088_ACCEL_CHIP_ID);
    bmi088SleepUs(BMI088_WAIT_TIME);
    accel_id = readAccelRegister(BMI088_ACCEL_CHIP_ID);
    bmi088SleepUs(BMI088_WAIT_TIME);
    if (accel_id != BMI088_ACCEL_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++) {
        writeAccelRegister(BMI088_accel_reg_config[write_reg_num][0], BMI088_accel_reg_config[write_reg_num][1]);
        bmi088SleepUs(BMI088_WAIT_TIME);

        res = readAccelRegister(BMI088_accel_reg_config[write_reg_num][0]);
        bmi088SleepUs(BMI088_WAIT_TIME);

        if (res != BMI088_accel_reg_config[write_reg_num][1]) {
            return BMI088_accel_reg_config[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}

uint8_t BMI088::accelSelfTest(void) {
    int16_t self_test_accel[2][3];
    uint8_t bufp[6] = {0};
    uint8_t res = 0;
    uint8_t write_reg_num = 0;
    uint8_t accel_id = 0;

    static const uint8_t BMI088_ACCEL_self_test_config[6][3] = {
        {BMI088_ACCEL_CONFIG,       BMI088_ACCEL_NORMAL | BMI088_ACCEL_1600_HZ | BMI088_ACCEL_CONFIG_MUST_SET,  BMI088_ACCEL_CONF_ERROR},
        {BMI088_ACCEL_PWR_CTRL,     BMI088_ACCEL_ENABLE_ACCEL_ON,                                               BMI088_ACCEL_PWR_CTRL_ERROR},
        {BMI088_ACCEL_RANGE,        BMI088_ACCEL_RANGE_24G,                                                     BMI088_ACCEL_RANGE_ERROR},
        {BMI088_ACCEL_PWR_CONFIG,   BMI088_ACCEL_PWR_ACTIVE_MODE,                                               BMI088_ACCEL_PWR_CONF_ERROR},
        {BMI088_ACCEL_SELF_TEST,    BMI088_ACCEL_SELF_TEST_POSITIVE_SIGNAL,                                     BMI088_ACCEL_SELF_TEST_ERROR},
        {BMI088_ACCEL_SELF_TEST,    BMI088_ACCEL_SELF_TEST_NEGATIVE_SIGNAL,                                     BMI088_ACCEL_SELF_TEST_ERROR}
    };

    accel_id = readAccelRegister(BMI088_ACCEL_CHIP_ID);
    bmi088SleepUs(BMI088_WAIT_TIME);
    accel_id = readAccelRegister(BMI088_ACCEL_CHIP_ID);
    bmi088SleepUs(BMI088_WAIT_TIME);
    if (accel_id != BMI088_ACCEL_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    writeAccelRegister(BMI088_ACCEL_SOFTRESET, BMI088_ACCEL_SOFTRESET_VALUE);
    bmi088SleepMs(BMI088_LONG_DELAY_TIME);

    accel_id = readAccelRegister(BMI088_ACCEL_CHIP_ID);
    bmi088SleepUs(BMI088_WAIT_TIME);
    accel_id = readAccelRegister(BMI088_ACCEL_CHIP_ID);
    bmi088SleepUs(BMI088_WAIT_TIME);
    if (accel_id != BMI088_ACCEL_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    for (write_reg_num = 0; write_reg_num < 4; write_reg_num++) {
        writeAccelRegister(BMI088_ACCEL_self_test_config[write_reg_num][0], BMI088_ACCEL_self_test_config[write_reg_num][1]);
        bmi088SleepUs(BMI088_WAIT_TIME);

        res = readAccelRegister(BMI088_ACCEL_self_test_config[write_reg_num][0]);
        bmi088SleepUs(BMI088_WAIT_TIME);

        if (res != BMI088_ACCEL_self_test_config[write_reg_num][1]) {
            return BMI088_ACCEL_self_test_config[write_reg_num][2];
        }
        bmi088SleepMs(BMI088_LONG_DELAY_TIME);
    }

    for (write_reg_num = 0; write_reg_num < 2; write_reg_num++) {
        writeAccelRegister(BMI088_ACCEL_self_test_config[write_reg_num + 4][0], BMI088_ACCEL_self_test_config[write_reg_num + 4][1]);
        bmi088SleepUs(BMI088_WAIT_TIME);

        res = readAccelRegister(BMI088_ACCEL_self_test_config[write_reg_num + 4][0]);
        bmi088SleepUs(BMI088_WAIT_TIME);

        if (res != BMI088_ACCEL_self_test_config[write_reg_num + 4][1]) {
            return BMI088_ACCEL_self_test_config[write_reg_num + 4][2];
        }
        bmi088SleepMs(BMI088_LONG_DELAY_TIME);
        
        readAccelMultiRegister(BMI088_ACCEL_XOUT_L, bufp, 6);
        self_test_accel[write_reg_num][0] = (int16_t)((bufp[1] << 8) | bufp[0]);
        self_test_accel[write_reg_num][1] = (int16_t)((bufp[3] << 8) | bufp[2]);
        self_test_accel[write_reg_num][2] = (int16_t)((bufp[5] << 8) | bufp[4]);
    }

    writeAccelRegister(BMI088_ACCEL_SELF_TEST, BMI088_ACCEL_SELF_TEST_OFF);
    bmi088SleepUs(BMI088_WAIT_TIME);
    res = readAccelRegister(BMI088_ACCEL_SELF_TEST);
    bmi088SleepUs(BMI088_WAIT_TIME);

    if (res != BMI088_ACCEL_SELF_TEST_OFF) {
        return BMI088_ACCEL_SELF_TEST_ERROR;
    }

    writeAccelRegister(BMI088_ACCEL_SOFTRESET, BMI088_ACCEL_SOFTRESET_VALUE);
    bmi088SleepMs(BMI088_LONG_DELAY_TIME);

    if ((self_test_accel[0][0] - self_test_accel[1][0]) < 1365 || (self_test_accel[0][1] - self_test_accel[1][1]) < 1365 || (self_test_accel[0][2] - self_test_accel[1][2]) < 680) {
        return BMI088_SELF_TEST_ACCEL_ERROR;
    }

    return BMI088_NO_ERROR;
}

uint8_t BMI088::gyroInit(void) {
    uint8_t res = 0;
    uint8_t write_reg_num = 0;
    uint8_t gyro_id = 0;

    gyro_id = readGyroRegister(BMI088_GYRO_CHIP_ID);
    bmi088SleepUs(BMI088_WAIT_TIME);
    gyro_id = readGyroRegister(BMI088_GYRO_CHIP_ID);
    bmi088SleepUs(BMI088_WAIT_TIME);
    if (gyro_id != BMI088_GYRO_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    writeGyroRegister(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    bmi088SleepMs(BMI088_LONG_DELAY_TIME);

    gyro_id = readGyroRegister(BMI088_GYRO_CHIP_ID);
    bmi088SleepUs(BMI088_WAIT_TIME);
    gyro_id = readGyroRegister(BMI088_GYRO_CHIP_ID);
    bmi088SleepUs(BMI088_WAIT_TIME);
    if (gyro_id != BMI088_GYRO_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++) {
        writeGyroRegister(BMI088_gyro_reg_config[write_reg_num][0], BMI088_gyro_reg_config[write_reg_num][1]);
        bmi088SleepUs(BMI088_WAIT_TIME);

        res = readGyroRegister(BMI088_gyro_reg_config[write_reg_num][0]);
        bmi088SleepUs(BMI088_WAIT_TIME);

        if (res != BMI088_gyro_reg_config[write_reg_num][1]) {
            return BMI088_gyro_reg_config[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}

uint8_t BMI088::gyroSelfTest(void) {
    uint8_t res = 0;
    uint8_t retry = 0;

    uint8_t gyro_id = readGyroRegister(BMI088_GYRO_CHIP_ID);
    bmi088SleepUs(BMI088_WAIT_TIME);
    if (gyro_id != BMI088_GYRO_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    writeGyroRegister(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    bmi088SleepMs(BMI088_LONG_DELAY_TIME);

    gyro_id = readGyroRegister(BMI088_GYRO_CHIP_ID);
    bmi088SleepUs(BMI088_WAIT_TIME);
    if (gyro_id != BMI088_GYRO_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    writeGyroRegister(BMI088_GYRO_SELF_TEST, BMI088_GYRO_TRIG_BIST);
    bmi088SleepMs(BMI088_LONG_DELAY_TIME);
    
    do {
        res = readGyroRegister(BMI088_GYRO_SELF_TEST);
        bmi088SleepUs(BMI088_WAIT_TIME);
        retry++;
    } while (!(res & BMI088_GYRO_BIST_RDY) && retry < 10);

    if (retry >= 10) {
        return BMI088_SELF_TEST_GYRO_ERROR;
    }

    if (res & BMI088_GYRO_BIST_FAIL) {
        return BMI088_SELF_TEST_GYRO_ERROR;
    }

    return BMI088_NO_ERROR;
}

uint8_t BMI088::readAccel(void) {
    uint8_t buf[6] = {0};
    readAccelMultiRegister(BMI088_ACCEL_XOUT_L, buf, 6);
    raw_data.accel_x = (int16_t)((buf[1] << 8) | buf[0]);
    raw_data.accel_y = (int16_t)((buf[3] << 8) | buf[2]);
    raw_data.accel_z = (int16_t)((buf[5] << 8) | buf[4]);

    real_data.accel_x = raw_data.accel_x * bmi088_accel_sen;
    real_data.accel_y = raw_data.accel_y * bmi088_accel_sen;
    real_data.accel_z = raw_data.accel_z * bmi088_accel_sen;

    return BMI088_NO_ERROR;
}

uint8_t BMI088::readGyro(void) {
    uint8_t buf[6] = {0};
    readGyroMultiRegister(BMI088_GYRO_X_L, buf, 6);
    raw_data.gyro_x = (int16_t)((buf[1] << 8) | buf[0]);
    raw_data.gyro_y = (int16_t)((buf[3] << 8) | buf[2]);
    raw_data.gyro_z = (int16_t)((buf[5] << 8) | buf[4]);

    real_data.gyro_x = raw_data.gyro_x * bmi088_gyro_sen;
    real_data.gyro_y = raw_data.gyro_y * bmi088_gyro_sen;
    real_data.gyro_z = raw_data.gyro_z * bmi088_gyro_sen;

    return BMI088_NO_ERROR;
}

uint8_t BMI088::readTemperature(void) {
    uint8_t buf[2] = {0};
    readAccelMultiRegister(BMI088_TEMP_M, buf, 2);
    raw_data.temperature = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (raw_data.temperature > 1023) {
        raw_data.temperature -= 2048;
    }
    real_data.temperature = raw_data.temperature * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

    return BMI088_NO_ERROR;
}

uint8_t BMI088::writeAccelRegister(uint8_t reg, uint8_t cmd) {
    uint8_t tx[2] = {reg, cmd};
    uint8_t rx[2] = {0};

    bmi088AccelCsLow();
    spiXfer(spiHandle, (char *)tx, (char *)rx, 2);
    bmi088AccelCsHigh();

    return 0x00;
}

uint8_t BMI088::readAccelRegister(uint8_t reg) {
    uint8_t tx[3] = {(uint8_t)(reg | 0x80), 0x55, 0x55};
    uint8_t rx[3] = {0};

    bmi088AccelCsLow();
    spiXfer(spiHandle, (char *)tx, (char *)rx, 3);
    bmi088AccelCsHigh();

    return rx[2];
}

uint8_t BMI088::readAccelMultiRegister(uint8_t reg, uint8_t *bufp, uint8_t len) {
    uint8_t tx[6 + 2] = {(uint8_t)(reg | 0x80), 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55};
    uint8_t rx[6 + 2] = {0};

    bmi088AccelCsLow();
    spiXfer(spiHandle, (char *)tx, (char *)rx, len + 2);
    bmi088AccelCsHigh();

    for (int i = 0; i < len; i++) {
        bufp[i] = rx[i + 2];
    }

    return 0x00;
}

uint8_t BMI088::writeGyroRegister(uint8_t reg, uint8_t cmd) {
    uint8_t tx[2] = {reg, cmd};
    uint8_t rx[2] = {0};

    bmi088GyroCsLow();
    spiXfer(spiHandle, (char *)tx, (char *)rx, 2);
    bmi088GyroCsHigh();

    return 0x00;
}

uint8_t BMI088::readGyroRegister(uint8_t reg) {
    uint8_t tx[2] = {(uint8_t)(reg | 0x80), 0x55};
    uint8_t rx[2] = {0};

    bmi088GyroCsLow();
    spiXfer(spiHandle, (char *)tx, (char *)rx, 2);
    bmi088GyroCsHigh();

    return rx[1];
}

uint8_t BMI088::readGyroMultiRegister(uint8_t reg, uint8_t *bufp, uint8_t len) {
    uint8_t tx[6 + 1] = {(uint8_t)(reg | 0x80), 0x55, 0x55, 0x55, 0x55, 0x55, 0x55};
    uint8_t rx[6 + 1] = {0};

    bmi088GyroCsLow();
    spiXfer(spiHandle, (char *)tx, (char *)rx, len + 1);
    bmi088GyroCsHigh();

    for (int i = 0; i < len; i++) {
        bufp[i] = rx[i + 1];
    }

    return 0x00;
}

void BMI088::bmi088CsInit(void) {
    gpioSetMode(csAccel, PI_OUTPUT);
    gpioSetMode(csGyro,  PI_OUTPUT);
    gpioWrite(csAccel, 1);
    gpioWrite(csGyro,  1);
}

void BMI088::bmi088AccelCsHigh(void) {
    gpioWrite(csAccel, 1);
}

void BMI088::bmi088AccelCsLow(void) {
    gpioWrite(csAccel, 0);
}

void BMI088::bmi088GyroCsHigh(void) {
    gpioWrite(csGyro, 1);
}

void BMI088::bmi088GyroCsLow(void) {
    gpioWrite(csGyro, 0);
}

void BMI088::bmi088SleepMs(unsigned int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void BMI088::bmi088SleepUs(unsigned int us) {
    std::this_thread::sleep_for(std::chrono::microseconds(us));
}
