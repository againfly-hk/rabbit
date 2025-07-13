#include "bmi088.h"
#include "controllerThread.h"
#include "threadAffinity.h"
#include "mahony_ahrs.h"
#include "pid.h"

#include <pigpio.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <pthread.h>
#include <fstream>

const int pwmFreq = 333;
const int pwmRange = 300;
const int pwmPins[4] = {17, 18, 14, 15};
const int pwmMid[4] = {148, 151, 158, 157};
const int servoPowerPin = 4;

const double imuAccelM[3][3]  = {{1.0089586369318100,       -0.00945613312305,      0.0095433941381734},
                                 {0.0098418181240635,       1.001964638861180,      -0.002937258914674},
                                 {-0.011077709387792,       0.016352307966567,      1.0086474541344300}};
const double imuAccelB[3] =      {-0.017690000000000,       0.058579000000000,      -0.002988000000000};
const double imuGyroE[3] =       {0.0002308070333333,       -0.00300226951667,      0.0001757681333333};

const int servoMartix[4][3] = {
    {-1, -1, +1},
    {-1, -1, -1},
    {+1, -1, -1},
    {+1, -1, +1}
};

int imuCount = 0;
int rollServoValue = 0;
int pitchServoValue = 0;
int yawServoValue = 0;
int pwmControl[4] = {0, 0, 0, 0};

double pitchControl = 0.0;
double rollControl = 0.0;
double yawControl = 0.0;

extern uint8_t detectFlag; 
extern int detectx;
extern int detectz;

BMI088RealData imuCalibratedData;

pid_type_def rollPid, pitchPid, yawPid;
pid_type_def dRollPid, dPitchPid, dYawPid;

void pidInit(void) {
    float rollPidParams[3] = {0.5, 0.1, 0};
    float pitchPidParams[3] = {0.5, 0.1, 0};
    float yawPidParams[3] = {0.5, 0.1, 0};

    PID_init(&rollPid, PID_POSITION, rollPidParams, 5.0f, 1.0f);
    PID_init(&pitchPid, PID_POSITION, pitchPidParams, 5.0f, 1.0f);
    PID_init(&yawPid, PID_POSITION, yawPidParams, 5.0f, 1.0f);

    float dRollPidParams[3] =   {1, 0, 0};
    float dPitchPidParams[3] =  {1, 0, 0};
    float dYawPidParams[3] =    {1, 0, 0};

    PID_init(&dRollPid, PID_DELTA, dRollPidParams, 30.0f, 10.0f);
    PID_init(&dPitchPid, PID_DELTA, dPitchPidParams, 30.0f, 10.0f);
    PID_init(&dYawPid, PID_DELTA, dYawPidParams, 30.0f, 10.0f);
}

void rabbitRollController(const uint8_t enableFlag) {
    if (enableFlag) {
        rollControl += PID_calc(&dRollPid, imuCalibratedData.gyro_x, 0.0f);
    } else {
        rollControl = 0.0;
    }
}

void rabbitPitchController(const uint8_t enableFlag) {
    if (enableFlag) {
        pitchControl += PID_calc(&dPitchPid, imuCalibratedData.accel_x, 0.0f);
    } else {
        pitchControl = 0.0;
    }
}

void rabbitYawController(const uint8_t enableFlag) {
    if (enableFlag) {
        PI
    } else {
        yawControl = 0.0;
    }
}

void rabbitServoController(void) {
    pwmControl[0] = pitchControl * servoMartix[0][0] + rollControl * servoMartix[0][1] + yawControl * servoMartix[0][2];
    pwmControl[1] = pitchControl * servoMartix[1][0] + rollControl * servoMartix[1][1] + yawControl * servoMartix[1][2];
    pwmControl[2] = pitchControl * servoMartix[2][0] + rollControl * servoMartix[2][1] + yawControl * servoMartix[2][2];
    pwmControl[3] = pitchControl * servoMartix[3][0] + rollControl * servoMartix[3][1] + yawControl * servoMartix[3][2];
    for (int i = 0; i < 4; i++) {
        if (pwmControl[i] > 50) {
            pwmControl[i] = 50;
        } else if (pwmControl[i] < -50) {
            pwmControl[i] = -50;
        }
        gpioPWM(pwmPins[i], pwmMid[i] + pwmControl[i]);
    }
}

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
    
    for (int i = 0; i < 4; i++) { 
        gpioSetMode(pwmPins[i], PI_OUTPUT);
        gpioSetPWMfrequency(pwmPins[i], pwmFreq);
        gpioSetPWMrange(pwmPins[i], pwmRange);
        gpioPWM(pwmPins[i], pwmMid[i]);
    }

    std::ofstream imuDataFile("imu_data.csv");
    imuDataFile << "dtime,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z" << std::endl;

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

            imuDataFile << imuCalibratedData.time << ","
                         << imuCalibratedData.accel_x << ","
                         << imuCalibratedData.accel_y << ","
                         << imuCalibratedData.accel_z << ","
                         << imuCalibratedData.gyro_x << ","
                         << imuCalibratedData.gyro_y << ","
                         << imuCalibratedData.gyro_z << std::endl;
            
            PID_calc(&yawPid, ahrs.yaw, 0.0f);
            PID_calc(&dYawPid, imuCalibratedData.gyro_z, yawPid.out);
            yawControl = dYawPid.out;
            rabbitServoController();
            // if (detectFlag) {
            //     rollServoValue = PID_calc(&rollPid, imuCalibratedData.accel_x, detectx);
            //     pitchServoValue = PID_calc(&pitchPid, imuCalibratedData.accel_y, detectz);
            //     yawServoValue = PID_calc(&yawPid, imuCalibratedData.gyro_z, 0.0f);

            //     rabbitRollController();
            //     rabbitPitchController(1);
            //     rabbitYawController(1);
            // } else {
            //     rabbitPitchController(0);
            //     rabbitYawController(0);
            // }
        } else {
            imuCount++;
            if (imuCount == 100) {
                std::cout << "IMU wait complete." << std::endl;
                lastTime = std::chrono::high_resolution_clock::now();
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    imuDataFile.close();
    gpioTerminate();
}