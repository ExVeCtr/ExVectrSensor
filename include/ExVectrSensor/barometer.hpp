#ifndef EXVECTRHAL_UTILITIES_TIMEHAL_H
#define EXVECTRHAL_UTILITIES_TIMEHAL_H

#include "stdint.h"

namespace VCTR
{

    /**
     * @brief A function to get the system time since start in nanoseconds.
     * @note Will overflow after ~292.47 years. If a deployed system runs longer then that and crashes then give me a call.
     *  
     * @returns time since system start in nanoseconds.
     */
    extern int64_t internalTime();

}

#endif