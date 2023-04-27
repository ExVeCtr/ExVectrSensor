#include "ExVectrSensor/barometer.hpp"

namespace VCTR
{

    const Core::Topic<Core::Timestamped<Data::ValueCov<float, 1>>> &SNSR::Barometer::getBaroTopic() const {
        return baroTopic_;
    }

}
