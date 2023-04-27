#include "ExVectrSensor/magnetometer.hpp"

namespace VCTR
{

    const Core::Topic<Core::Timestamped<Data::ValueCov<float, 3>>> &SNSR::Magnetometer::getMagTopic() const {
        return magTopic_;
    }

}
