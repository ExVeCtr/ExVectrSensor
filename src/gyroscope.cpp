#include "ExVectrSensor/gyroscope.hpp"

namespace VCTR
{

    Core::Topic<Core::Timestamped<Data::ValueCov<float, 3>>> &SNSR::Gyroscope::getGyroTopic() {
        return gyroTopic_;
    }

}
