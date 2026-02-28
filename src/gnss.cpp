#include "ExVectrSensor/gnss.hpp"

namespace VCTR::sensor {

Core::Topic<GNSSDataStamped> &GNSS::getGNSSTopic() { return gnssTopic_; }

} // namespace VCTR::sensor
