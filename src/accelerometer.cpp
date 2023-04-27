#include "ExVectrSensor/accelerometer.hpp"

namespace VCTR
{

    const Core::Topic<Core::Timestamped<Data::ValueCov<float, 3>>> &SNSR::Accelerometer::getAccelTopic() const {
        return accelTopic_;
    }

}
