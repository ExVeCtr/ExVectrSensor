#include "ExVectrSensor/thermometer.hpp"

namespace VCTR::sensor {

Core::Topic<ThermoDataStamped> &Thermometer::getThermoTopic() {
  return thermTopic_;
}

} // namespace VCTR::sensor
