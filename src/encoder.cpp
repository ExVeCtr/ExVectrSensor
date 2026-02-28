#include "ExVectrSensor/encoder.hpp"

namespace VCTR::sensor {

Core::Topic<EncoderDataStamped> &Encoder::getEncoderTopic() {
  return encoderTopic_;
}

} // namespace VCTR::sensor
