#ifndef CONTROLLER_THREAD_H
#define CONTROLLER_THREAD_H

#include <atomic>

void controllerThreadFunc(std::atomic<bool>& keepRunning);

#endif
