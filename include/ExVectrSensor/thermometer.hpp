#ifndef EXVECTRSENSOR_THERMOMETER_H
#define EXVECTRSENSOR_THERMOMETER_H

#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrDSP/value_covariance.hpp"

namespace VCTR::sensor {
using ThermoDataStamped = Core::Timestamped<DSP::ValueCov<float, 1>>;

/**
 * @brief An abstract class for thermometers. Allows things like sensorfusion to
 * use any thermometer class so long this is inhereted from.
 */
class Thermometer {
protected:
  /// @brief Topic to which new thermometer values should be published in
  /// degrees celsius.
  Core::Topic<ThermoDataStamped> thermTopic_;

public:
  /**
   * @brief Gets the thermometer topic where new thermometer values are
   * published in degrees celsius.
   * @returns thermometer topic.
   */
  Core::Topic<ThermoDataStamped> &getThermoTopic();

  /**
   * @brief Makes the sensor read the thermometer and publish the sensor values.
   * @note Implemented by child class.
   * @return true if reading was successfull. False otherwise.
   */
  virtual bool readTherm() = 0;
};

} // namespace VCTR::sensor

#endif