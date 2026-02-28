#include "ExVectrSensor/accelerometer.hpp"

namespace VCTR::sensor {

Core::Topic<AccelDataStamped> &Accelerometer::getAccelTopic() {
  return accelTopic_;
}

} // namespace VCTR::sensor
