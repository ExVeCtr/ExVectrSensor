#include "ExVectrSensor/magnetometer.hpp"

namespace VCTR::sensor {

Core::Topic<MagDataStamped> &Magnetometer::getMagTopic() { return magTopic_; }

} // namespace VCTR::sensor
