#pragma once

#include <chrono>
#include <cstdint>


struct rosTime
{
    uint32_t sec = 0, nsec = 0;
};
struct rosDuration
{
    uint32_t sec = 0, nsec = 0;
};

inline rosTime rosTimeNow()
{
    auto n = std::chrono::high_resolution_clock::now();
    uint64_t ns = std::duration_cast<std::chrono::nanoseconds>(n.time_since_epoch()).count();
    rosTime r;
    r.sec = ns / 1000000000;
    r.nsec = ns % 1000000000;
    return r;
}
