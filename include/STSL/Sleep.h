#ifndef SOFTWARETRAININGSUPPORTLIBRARY_SLEEP_H
#define SOFTWARETRAININGSUPPORTLIBRARY_SLEEP_H

#ifdef __WIN32
#include <windows.h>
#elif __linux__
#include <unistd.h>
#elif __APPLE__
#include <unistd.h>
#endif

#include <chrono>

inline void sleep(std::chrono::microseconds duration) {
    #ifdef __WIN32
    ::Sleep(static_cast<DWORD>(std::chrono::duration_cast<std::chrono::milliseconds>(duration).count()));
    #elif __linux__
    usleep(static_cast<__useconds_t>(duration.count()));
    #elif __APPLE__
    usleep(static_cast<useconds_t>(duration.count()));
    #endif
}

#endif //SOFTWARETRAININGSUPPORTLIBRARY_SLEEP_H

