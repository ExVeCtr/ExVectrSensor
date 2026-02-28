#ifndef EXVECTRSENSOR_BAROMETER_H
#define EXVECTRSENSOR_BAROMETER_H

#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrDSP/value_covariance.hpp"

namespace VCTR::sensor {

using BaroDataStamped = Core::Timestamped<DSP::ValueCov<float, 1>>;

/**
 * @brief An abstract class for barometers. Allows things like sensorfusion to
 * use any barometer class so long this is inhereted from.
 */
class Barometer {
protected:
  /// @brief Topic to which new barometer values should be published in Pascal
  /// in sensor frame.
  Core::Topic<BaroDataStamped> baroTopic_;

public:
  /**
   * @brief Gets the barometer topic where new barometer values are published in
   * Pascal in sensor frame.
   * @returns barometer topic.
   */
  Core::Topic<BaroDataStamped> &getBaroTopic();

  /**
   * @brief Makes the sensor read the barometer and publish the sensor values.
   * @note Implemented by child class.
   * @return true if reading was successfull. False otherwise.
   */
  virtual bool readBaro() = 0;
};

} // namespace VCTR::sensor

#endif