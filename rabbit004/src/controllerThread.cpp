#include "bmi088.h"
#include "controllerThread.h"
#include "threadAffinity.h"
#include "mahony_ahrs.h"

#include <pigpio.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <pthread.h>

const int pwmFreq = 333;
const int pwmRange = 300;
const int pwmPins[4] = {17, 18, 14, 15};
const int servoPowerPin = 4;

const double imuAccelM[3][3]  = {{1.0089586369318100,       -0.00945613312305,      0.0095433941381734},
                                 {0.0098418181240635,       1.001964638861180,      -0.002937258914674},
                                 {-0.011077709387792,       0.016352307966567,      1.0086474541344300}};
const double imuAccelB[3] =      {-0.017690000000000,       0.058579000000000,      -0.002988000000000};
const double imuGyroE[3] =       {0.0002308070333333,       -0.00300226951667,      0.0001757681333333};
int imuCount = 0;

BMI088RealData imuCalibratedData;

void controllerThreadFunc(std::atomic<bool>& keepRunning) {
    setCurrentThreadAffinity(2, "controllerThread");

    BMI088 imu;
    MahonyAHRS ahrs;

    if (gpioInitialise() < 0) {
        std::cerr << "GPIO initialization failed!" << std::endl;
        return;
    }

    gpioSetMode(servoPowerPin, PI_OUTPUT);
    gpioWrite(servoPowerPin, 1);
    
    for (int pin : pwmPins) { 
        gpioSetMode(pin, PI_OUTPUT);
        gpioSetPWMfrequency(pin, pwmFreq);
        gpioSetPWMrange(pin, pwmRange);
        gpioPWM(pin, 150);
    }

    auto lastTime = std::chrono::high_resolution_clock::now();
    while (keepRunning.load()) {
        imu.readAccel();
        imu.readGyro();
        if(imuCount >= 100) {
            double calibratedAccel_x = imu.real_data.accel_x - imuAccelB[0];
            double calibratedAccel_y = imu.real_data.accel_y - imuAccelB[1];
            double calibratedAccel_z = imu.real_data.accel_z - imuAccelB[2];

            imuCalibratedData.accel_x = imuAccelM[0][0] * calibratedAccel_x + imuAccelM[0][1] * calibratedAccel_y + imuAccelM[0][2] * calibratedAccel_z + imuAccelB[0];
            imuCalibratedData.accel_y = imuAccelM[1][0] * calibratedAccel_x + imuAccelM[1][1] * calibratedAccel_y + imuAccelM[1][2] * calibratedAccel_z + imuAccelB[1];
            imuCalibratedData.accel_z = imuAccelM[2][0] * calibratedAccel_x + imuAccelM[2][1] * calibratedAccel_y + imuAccelM[2][2] * calibratedAccel_z + imuAccelB[2];

            imuCalibratedData.gyro_x = imu.real_data.gyro_x - imuGyroE[0];
            imuCalibratedData.gyro_y = imu.real_data.gyro_y - imuGyroE[1];
            imuCalibratedData.gyro_z = imu.real_data.gyro_z - imuGyroE[2];

            auto nowTime = std::chrono::high_resolution_clock::now();
            imuCalibratedData.time = std::chrono::duration_cast<std::chrono::microseconds>(nowTime - lastTime).count() * 1e-6;
            lastTime = nowTime;

            ahrs.updateIMU(imuCalibratedData.gyro_x, imuCalibratedData.gyro_y, imuCalibratedData.gyro_z,
                           imuCalibratedData.accel_x, imuCalibratedData.accel_y, imuCalibratedData.accel_z,
                           imuCalibratedData.time);

            std::cout << "IMU Data: "
                      << "pitch: " << ahrs.pitch << ", "
                      << "roll: " << ahrs.roll << ", "
                      << "yaw: " << ahrs.yaw << std::endl;
        } else {
            imuCount++;
            if (imuCount == 100) {
                std::cout << "IMU wait complete." << std::endl;
                lastTime = std::chrono::high_resolution_clock::now();
            }
        }


        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    gpioTerminate();
}