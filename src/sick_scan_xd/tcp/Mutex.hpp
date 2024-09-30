/**
 * \file Mutex.hpp
 */

#pragma once

#include <mutex>
#include <thread>

#include "BasicDatatypes.hpp"
// #include <pthread.h>


//
// Mutex class
//
class Mutex
{
public:
    Mutex();
    ~Mutex();

    void lock();
    void unlock();

private:
    std::mutex m_mutex;
    // pthread_mutex_t m_mutex;
};



//
// Scoped Lock.
// Zerstoert das Mutex automatisch.
//
class ScopedLock
{
public:
    ScopedLock(Mutex* mutexPtr);
    ~ScopedLock();
private:
    Mutex* m_mutexPtr;

};
