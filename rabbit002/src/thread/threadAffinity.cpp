#include "threadAffinity.h"

#include <pthread.h>
#include <sched.h>
#include <iostream>

void setCurrentThreadAffinity(int cpu_id, const std::string& thread_name) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_id, &cpuset);

    pthread_t current_thread = pthread_self();
    int rc = pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);
    if (rc != 0) {
        std::cerr << "Error setting CPU affinity for thread " << thread_name
                  << ": " << rc << std::endl;
    } else {
        std::cout << "Thread " << thread_name << " bound to CPU " << cpu_id << std::endl;
    }
}
