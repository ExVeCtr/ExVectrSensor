#ifndef EXVECTRSENSOR_ENCODER_H
#define EXVECTRSENSOR_ENCODER_H

#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrDSP/value_covariance.hpp"

namespace VCTR::sensor {

using EncoderDataStamped = Core::Timestamped<DSP::ValueCov<int32_t, 1>>;

/**
 * @brief An abstract class for encoders. Allows things like sensorfusion to use
 * any encoder class so long this is inhereted from.
 * @note Publishing a value to the encoder topic will set the encoder to the
 * value.
 */
class Encoder {
protected:
  /// @brief Topic to which new encoder values should be published in counts.
  Core::Topic<EncoderDataStamped> encoderTopic_;

public:
  /**
   * @brief Gets the encoder topic where new encoder values are published in
   * counts.
   * @returns encoder topic.
   */
  Core::Topic<EncoderDataStamped> &getEncoderTopic();

  /**
   * @brief Makes the sensor read the values and publish them to the topic.
   * @note Implemented by child class.
   * @return true if reading was successfull. False otherwise.
   */
  virtual bool readEncoder() = 0;
};

} // namespace VCTR::sensor

#endif