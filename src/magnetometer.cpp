#include "ExVectrSensor/magnetometer.hpp"

namespace VCTR
{

    Core::Topic<Core::Timestamped<Data::ValueCov<float, 3>>> &SNSR::Magnetometer::getMagTopic() {
        return magTopic_;
    }

}
