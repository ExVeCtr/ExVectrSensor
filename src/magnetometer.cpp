#include "ExVectrSensor/magnetometer.hpp"

namespace VCTR
{

    Core::Topic<Core::Timestamped<DSP::ValueCov<float, 3>>> &SNSR::Magnetometer::getMagTopic() {
        return magTopic_;
    }

}
