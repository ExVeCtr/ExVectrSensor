#include "ExVectrSensor/accelerometer.hpp"

namespace VCTR
{

    Core::Topic<Core::Timestamped<DSP::ValueCov<float, 3>>> &SNSR::Accelerometer::getAccelTopic() {
        return accelTopic_;
    }

}
