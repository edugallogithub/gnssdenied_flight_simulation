#ifndef MATH_TIMER
#define MATH_TIMER

#include "../math.h"
#include <chrono>

namespace math {

// CLASS TIMER
// ===========
// ===========

class MATH_API timer {
private:
    /**< time stamp for the last execution of the method "start" */
    std::chrono::high_resolution_clock::time_point _start_time;
    /**< time stamp for the last execution of the method "stop" */
    std::chrono::high_resolution_clock::time_point _stop_time;
public:
    /**< starts the clock, time stamping the current time */
    void start() {_start_time = std::chrono::high_resolution_clock::now();}
    /**< stops the clock, time stamping the current time */
    void stop() {_stop_time = std::chrono::high_resolution_clock::now();}
    /**< returns the time difference [nanoseconds] between the last time
    the "stop" and "start" methods were executed */
    double measure() const {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(_stop_time - _start_time).count();
    }
}; // closes class timer

} // closes namespace math

#endif


