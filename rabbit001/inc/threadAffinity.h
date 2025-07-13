#ifndef THREAD_AFFINITY_H
#define THREAD_AFFINITY_H

#include <string>

void setCurrentThreadAffinity(int cpu_id, const std::string& thread_name);

#endif
