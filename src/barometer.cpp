#include "ExVectrSensor/barometer.hpp"

namespace VCTR
{

    Core::Topic<Core::Timestamped<DSP::ValueCov<float, 1>>> &SNSR::Barometer::getBaroTopic() {
        return baroTopic_;
    }

}
