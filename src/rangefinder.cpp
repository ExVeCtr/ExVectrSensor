#include "ExVectrSensor/rangefinder.hpp"

namespace VCTR
{

    Core::Topic<Core::Timestamped<DSP::ValueCov<float, 1>>> &SNSR::RangeFinder::getRangeTopic()
    {
        return rangeTopic_;
    }

}
