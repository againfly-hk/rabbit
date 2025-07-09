#include "visionData.h"
#include "visionThread.h"
#include "controllerThread.h"

#include <thread>
#include <atomic>
#include <iostream>
#include <csignal>

std::atomic<bool> keepRunning(true);

void signalHandler(int signum) {
    std::cout << std::endl << "Received signal " << signum << ", exiting..." << std::endl;
    keepRunning = false;
}

int main() {
    VisionQueue visionQueue;

    std::signal(SIGINT, signalHandler);
    std::thread visionThread(visionThreadFunc, std::ref(keepRunning), std::ref(visionQueue));
    std::thread controllerThread(controllerThreadFunc, std::ref(keepRunning), std::ref(visionQueue));

    visionThread.join();
    controllerThread.join();

    return 0;
}