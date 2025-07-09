#include "controllerThread.h"
#include "bmi088.h"

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

void controllerThreadFunc(std::atomic<bool>& keepRunning, VisionQueue& visionQueue) {
    bindControllerThreadToCPU(1);

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
        if (!visionQueue.empty()) {
            Point p = visionQueue.wait_and_pop();
            // Process the point p as needed
            std::cout << "Received point: (" << p.x << ", " << p.y << ")" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    gpioTerminate();
}