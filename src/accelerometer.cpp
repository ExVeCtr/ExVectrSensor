#include "ExVectrSensor/accelerometer.hpp"

namespace VCTR
{

    Core::Topic<Core::Timestamped<Data::ValueCov<float, 3>>> &SNSR::Accelerometer::getAccelTopic() {
        return accelTopic_;
    }

}
