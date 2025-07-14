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

/* servo pwm set */
const int pwmFreq = 333;
const int pwmRange = 300;
const int servoPowerPin = 4;
const int pwmPins[4] = {17, 18, 14, 15};
const int pwmMid[4] = {148, 151, 158, 157};
int pwmControl[4] = {0, 0, 0, 0};
const int servoMartix[4][3] = {
    {-1, -1, +1},
    {-1, -1, -1},
    {+1, -1, -1},
    {+1, -1, +1}
};

/* imu calibration */
int imuCount = 0;
BMI088 imu;
MahonyAHRS ahrs;
BMI088RealData imuCalibratedData;
const double imuAccelA[3][3]  = {{0.00984181812406,   1.00196463886118,     -0.0029372589147},
                                 {-0.0110777093878,   0.01635230796657,     1.00864745413443},
                                 {1.00895863693181,   -0.0094561331230,     0.00954339413817}};
const double imuAccelB[3] =      {-0.0176900000000,   0.05857900000000,     -0.0029880000000};
const double imuGyroA[3][3]  =  {{0.00000000000000,   1.00000000000000,     0.00000000000000},
                                 {0.00000000000000,   0.00000000000000,     1.00000000000000},
                                 {1.00000000000000,   0.00000000000000,     0.00150000000000}};
const double imuGyroB[3] =       {0.00023080703333,   -0.0030022695167,     0.00017576813333};

/* pid */
double pitchControl = 0.0;
double rollControl = 0.0;
double yawControl = 0.0;
pid_type_def rollPid, pitchPid, yawPid;
pid_type_def dRollPid, dPitchPid, dYawPid;

/* vision */
extern uint8_t  detectflag; 
extern int      detectz;
extern int      detecty;

/* lqr control */
double flyTime  = 0.0;
double startFlyTime = 0.0;
uint8_t flyFlag = 0;

void pidInit(void) {
    float rollPidParams[3] =    {5, 0, 0};
    float pitchPidParams[3] =   {5, 0, 0};
    float yawPidParams[3] =     {5, 0, 0};

    PID_init(&rollPid,  PID_POSITION, rollPidParams,    1.0f, 0.5f);
    PID_init(&pitchPid, PID_POSITION, pitchPidParams,   1.0f, 0.5f);
    PID_init(&yawPid,   PID_POSITION, yawPidParams,     1.0f, 0.5f);

    float dRollPidParams[3] =   {100, 1, 0};
    float dPitchPidParams[3] =  {100, 1, 0};
    float dYawPidParams[3] =    {100, 1, 0};

    PID_init(&dRollPid,     PID_POSITION, dRollPidParams,   50.0f, 10.0f);
    PID_init(&dPitchPid,    PID_POSITION, dPitchPidParams,  50.0f, 10.0f);
    PID_init(&dYawPid,      PID_POSITION, dYawPidParams,    50.0f, 10.0f);
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
        double yaw_set = ahrs.yaw;
        yawControl += PID_calc(&dYawPid, imuCalibratedData.gyro_z, 0.0f);
        if (yawControl > 50) {
            yawControl = 50;
        } else if (yawControl < -50) {
            yawControl = -50;
        }
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
    imuDataFile << "dtime,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,yaw,pitch,roll" << std::endl;

    auto lastTime = std::chrono::high_resolution_clock::now();
    while (keepRunning.load()) {
        imu.readAccel();
        imu.readGyro();
        if(imuCount >= 100) {
            double calibratedbAccel_x = imu.real_data.accel_x - imuAccelB[0];
            double calibratedbAccel_y = imu.real_data.accel_y - imuAccelB[1];
            double calibratedbAccel_z = imu.real_data.accel_z - imuAccelB[2];

            double calibratedbGyro_x = imu.real_data.gyro_x - imuGyroB[0];
            double calibratedbGyro_y = imu.real_data.gyro_y - imuGyroB[1];
            double calibratedbGyro_z = imu.real_data.gyro_z - imuGyroB[2];

            imuCalibratedData.accel_x = imuAccelA[0][0] * calibratedbAccel_x + imuAccelA[0][1] * calibratedbAccel_y + imuAccelA[0][2] * calibratedbAccel_z;
            imuCalibratedData.accel_y = imuAccelA[1][0] * calibratedbAccel_x + imuAccelA[1][1] * calibratedbAccel_y + imuAccelA[1][2] * calibratedbAccel_z;
            imuCalibratedData.accel_z = imuAccelA[2][0] * calibratedbAccel_x + imuAccelA[2][1] * calibratedbAccel_y + imuAccelA[2][2] * calibratedbAccel_z;

            imuCalibratedData.gyro_x = imuGyroA[0][0] * calibratedbGyro_x + imuGyroA[0][1] * calibratedbGyro_y + imuGyroA[0][2] * calibratedbGyro_z;
            imuCalibratedData.gyro_y = imuGyroA[1][0] * calibratedbGyro_x + imuGyroA[1][1] * calibratedbGyro_y + imuGyroA[1][2] * calibratedbGyro_z;
            imuCalibratedData.gyro_z = imuGyroA[2][0] * calibratedbGyro_x + imuGyroA[2][1] * calibratedbGyro_y + imuGyroA[2][2] * calibratedbGyro_z;

            auto nowTime = std::chrono::high_resolution_clock::now();
            imuCalibratedData.time = std::chrono::duration_cast<std::chrono::microseconds>(nowTime - lastTime).count() * 1e-6;
            lastTime = nowTime;

            ahrs.updateIMU(imuCalibratedData.gyro_x, imuCalibratedData.gyro_y, imuCalibratedData.gyro_z,
                           imuCalibratedData.accel_x, imuCalibratedData.accel_y, imuCalibratedData.accel_z,
                           imuCalibratedData.time);
            
            std::cout << "Yaw: " << ahrs.yaw << ", Pitch: " << ahrs.pitch << ", Roll: " << ahrs.roll << std::endl;
            
            imuDataFile << imuCalibratedData.time       << ","
                        << imuCalibratedData.accel_x    << ","
                        << imuCalibratedData.accel_y    << ","
                        << imuCalibratedData.accel_z    << ","
                        << imuCalibratedData.gyro_x     << ","
                        << imuCalibratedData.gyro_y     << ","
                        << imuCalibratedData.gyro_z     << ","
                        << ahrs.yaw                     << ","
                        << ahrs.pitch                   << ","
                        << ahrs.roll                    << ","
                        << std::endl;
            if (flyFlag) {
                flyTime = std::chrono::duration_cast<std::chrono::microseconds>(nowTime).count() * 1e-6 - startFlyTime;
                std::cout << "Flight time: " << flyTime << " seconds" << std::endl;
                if(flyTime > 4.0) {
                    flyFlag = 0;
                    gpioWrite(servoPowerPin, 0);
                    std::cout << "Flight time exceeded 4 seconds, stopping flight." << std::endl;
                    break;
                }
                // Control logic based on vision detection
                
            } else {
                if(imuCalibratedData.accel_x > 2.5) {
                    flyFlag = 1;
                    gpioWrite(servoPowerPin, 1);
                    yawControl = 0;
                    pitchControl = 0;
                    rollControl = 0;
                    rabbitServoController();
                    startFlyTime = std::chrono::duration_cast<std::chrono::microseconds>(nowTime).count() * 1e-6;
                }
            }
        } else {
            imuCount++;
            if (imuCount == 100) {
                std::cout << "IMU wait complete." << std::endl;
                lastTime = std::chrono::high_resolution_clock::now();
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(9));
    }

    imuDataFile.close();
    gpioTerminate();
}