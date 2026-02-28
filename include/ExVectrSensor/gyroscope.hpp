#ifndef EXVECTRSENSOR_GYROSCOPE_H
#define EXVECTRSENSOR_GYROSCOPE_H

#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrDSP/value_covariance.hpp"

namespace VCTR::sensor {

using GyroDataStamped = Core::Timestamped<DSP::ValueCov<float, 3>>;

/**
 * @brief An abstract class for gyroscopes. Allows things like sensorfusion to
 * use any gyroscope class so long this is inhereted from.
 */
class Gyroscope {
protected:
  /// @brief Topic to which new gyroscope values should be published in [rad/s]
  /// in sensor frame.
  Core::Topic<GyroDataStamped> gyroTopic_;

public:
  /**
   * @brief Gets the gyroscope topic where new gyroscope values are published in
   * [rad/s] in sensor frame.
   * @returns gyroscope topic.
   */
  Core::Topic<GyroDataStamped> &getGyroTopic();

  /**
   * @brief Makes the sensor read the values and publish them to the topic.
   * @note Implemented by child class.
   * @return true if reading was successfull. False otherwise.
   */
  virtual bool readGyro() = 0;
};

} // namespace VCTR::sensor

#endif