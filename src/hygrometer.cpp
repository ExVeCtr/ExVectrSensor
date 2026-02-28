#include "ExVectrSensor/hygrometer.hpp"

namespace VCTR::sensor {

Core::Topic<HygroDataStamped> &Hygrometer::getHygroTopic() {
  return hygroTopic_;
}

} // namespace VCTR::sensor
