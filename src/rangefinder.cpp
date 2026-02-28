#include "ExVectrSensor/rangefinder.hpp"

namespace VCTR::sensor {

Core::Topic<RangeDataStamped> &RangeFinder::getRangeTopic() {
  return rangeTopic_;
}

} // namespace VCTR::sensor
