#ifndef VISION_THREAD_H
#define VISION_THREAD_H

#include <atomic>

void visionThreadFunc(std::atomic<bool>& keepRunning);
void imageProcessingThreadFunc(std::atomic<bool>& keepRunning);

#endif
