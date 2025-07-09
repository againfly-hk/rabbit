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

void bindMainThreadToCPU(int cpu_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_id, &cpuset);

    pthread_t current_thread = pthread_self();
    int rc = pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);
    if (rc != 0) {
        std::cerr << "Error calling pthread_setaffinity_np: " << rc << std::endl;
    } else {
        std::cout << "Main thread bound to CPU " << cpu_id << std::endl;
    }
}

int main() {
    bindMainThreadToCPU(1);
    VisionQueue visionQueue;

    std::signal(SIGINT, signalHandler);
    std::thread visionThread(visionThreadFunc, std::ref(keepRunning));
    std::thread imageProcessingThread(imageProcessingThreadFunc, std::ref(keepRunning));
    std::thread controllerThread(controllerThreadFunc, std::ref(keepRunning));

    visionThread.join();
    controllerThread.join();

    return 0;
}