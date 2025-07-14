#include "visionThread.h"
#include "controllerThread.h"
#include "threadAffinity.h"

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
    setCurrentThreadAffinity(2, "mainThread");

    std::signal(SIGINT, signalHandler);
    std::thread visionThread(visionThreadFunc, std::ref(keepRunning));
    std::thread imageProcessingThread(imageProcessingThreadFunc, std::ref(keepRunning));
    std::thread controllerThread(controllerThreadFunc, std::ref(keepRunning));

    visionThread.join();
    imageProcessingThread.join();
    controllerThread.join();

    return 0;
}