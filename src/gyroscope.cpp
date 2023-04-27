#include "ExVectrSensor/gyroscope.hpp"

namespace VCTR
{

    const Core::Topic<Core::Timestamped<Data::ValueCov<float, 3>>> &SNSR::Gyroscope::getGyroTopic() const {
        return gyroTopic_;
    }

}
