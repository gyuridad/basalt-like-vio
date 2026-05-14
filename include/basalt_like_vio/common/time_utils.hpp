
#pragma once  
#include <cstdint>  

namespace basalt_like_vio {
namespace common {

using TimeNs = int64_t;

inline TimeNs secondsToNanoseconds(double seconds) {
    return static_cast<TimeNs>(seconds * 1e9);
}

inline double nanosecondsToSeconds(TimeNs time_ns) {
    return static_cast<double>(time_ns) * 1e-9;
}

inline double deltaTimeSeconds(TimeNs newer_time_ns, TimeNs older_time_ns) {
    return nanosecondsToSeconds(newer_time_ns - older_time_ns);
}

}
}


