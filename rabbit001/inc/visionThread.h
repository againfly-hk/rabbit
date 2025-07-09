#ifndef VISION_THREAD_H
#define VISION_THREAD_H

#include "visionData.h"

#include <atomic>

void  visionThreadFunc(std::atomic<bool>& keepRunning, VisionQueue& visionQueue);

#endif
