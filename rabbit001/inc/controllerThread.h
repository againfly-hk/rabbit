#ifndef CONTROLLER_THREAD_H
#define CONTROLLER_THREAD_H

#include "visionData.h"

#include <atomic>

void controllerThreadFunc(std::atomic<bool>& keepRunning, VisionQueue& visionQueue);

#endif
