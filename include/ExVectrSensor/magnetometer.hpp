#ifndef EXVECTRSENSOR_MAGNETOMETER_H
#define EXVECTRSENSOR_MAGNETOMETER_H

#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrDSP/value_covariance.hpp"

namespace VCTR::sensor {

using MagDataStamped = Core::Timestamped<DSP::ValueCov<float, 3>>;

/**
 * @brief An abstract class for magnetometers. Allows things like sensorfusion
 * to use any magnetometer class so long this is inherited from.
 */
class Magnetometer {
protected:
  /// @brief Topic to which new magnetometer values should be published in
  /// [tesla] in sensor frame.
  Core::Topic<MagDataStamped> magTopic_;

public:
  /**
   * @brief Gets the magnetometer topic where new magnetometer values are
   * published in [tesla] in sensor frame.
   * @returns magnetometer topic.
   */
  Core::Topic<MagDataStamped> &getMagTopic();

  /**
   * @brief Makes the sensor read the values and publish them to the topic.
   * @note Implemented by child class.
   * @return true if reading was successfull. False otherwise.
   */
  virtual bool readMag() = 0;
};
} // namespace VCTR::sensor

#endif // EXVECTRSENSOR_MAGNETOMETER_H