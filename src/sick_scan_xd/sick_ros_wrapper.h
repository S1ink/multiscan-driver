#pragma once

#include <chrono>
#include <thread>
#include <cstdint>
#include <iostream>


#define ROS_DEBUG_STREAM(x) /*std::cout << x << std::endl;*/
#define ROS_INFO_STREAM(x) std::cout << x << std::endl;
#define ROS_WARN_STREAM(x) std::cout << x << std::endl;
#define ROS_ERROR_STREAM(x) std::cout << x << std::endl;
#define ROS_DEBUG(x)
#define ROS_INFO(x)
#define ROS_WARN(x)
#define ROS_ERROR(x)

static inline constexpr bool rosOk() { return true; }


struct rosTime
{
    uint32_t sec = 0, nsec = 0;
};
struct rosDuration
{
    uint32_t sec = 0, nsec = 0;
};

inline uint64_t rosNanosecTimestampNow()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
}
inline rosTime rosTimeNow()
{
    uint64_t ns = rosNanosecTimestampNow();
    rosTime r;
    r.sec = ns / 1000000000;
    r.nsec = ns % 1000000000;
    return r;
}

inline uint32_t sec(rosTime t)
{
    return t.sec;
}
inline uint32_t nsec(rosTime t)
{
    return t.nsec;
}

inline uint32_t sec(rosDuration t)
{
    return t.sec;
}
inline uint32_t nsec(rosDuration t)
{
    return t.nsec;
}

inline void rosSleep(double seconds)
{
    std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
}
