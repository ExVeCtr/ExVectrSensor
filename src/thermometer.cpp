#include "ExVectrSensor/thermometer.hpp"

namespace VCTR
{

    Core::Topic<Core::Timestamped<Data::ValueCov<float, 1>>> &SNSR::Thermometer::getThermoTopic() {
        return thermTopic_;
    }

}
