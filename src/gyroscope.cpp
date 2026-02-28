#include "ExVectrSensor/gyroscope.hpp"

namespace VCTR::sensor {

Core::Topic<GyroDataStamped> &Gyroscope::getGyroTopic() { return gyroTopic_; }

} // namespace VCTR::sensor
