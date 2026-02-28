#include "ExVectrSensor/barometer.hpp"

namespace VCTR::sensor {

Core::Topic<BaroDataStamped> &Barometer::getBaroTopic() { return baroTopic_; }

} // namespace VCTR::sensor