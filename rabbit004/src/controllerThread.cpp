#include "bmi088.h"
#include "controllerThread.h"
#include "threadAffinity.h"

#include <pigpio.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <pthread.h>

const int pwmFreq = 333;
const int pwmRange = 300;
const int pwmPins[4] = {17, 18, 14, 15};

void bindControllerThreadToCPU(int cpu_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_id, &cpuset);

    pthread_t current_thread = pthread_self();
    int rc = pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);
    if (rc != 0) {
        std::cerr << "Error calling pthread_setaffinity_np: " << rc << std::endl;
    } else {
        std::cout << "Controller thread bound to CPU " << cpu_id << std::endl;
    }
}

void controllerThreadFunc(std::atomic<bool>& keepRunning) {
    setCurrentThreadAffinity(2, "controllerThread");

    if (gpioInitialise() < 0) {
        std::cerr << "GPIO initialization failed!" << std::endl;
        return;
    }
    
    for (int pin : pwmPins) { 
        gpioSetMode(pin, PI_OUTPUT);
        gpioSetPWMfrequency(pin, pwmFreq);
        gpioSetPWMrange(pin, pwmRange);
        gpioPWM(pin, 150);
    }

    while (keepRunning.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    gpioTerminate();
}