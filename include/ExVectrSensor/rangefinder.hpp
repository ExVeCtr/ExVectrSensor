#ifndef EXVECTRSENSOR_RANGEFINDER_H
#define EXVECTRSENSOR_RANGEFINDER_H

#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrDSP/value_covariance.hpp"

namespace VCTR {

namespace sensor {

using RangeDataStamped = Core::Timestamped<DSP::ValueCov<float, 1>>;

/**
 * @brief An abstract class for rangefinders. Allows things like sensorfusion to
 * use any rangefinder class so long this is inhereted from.
 */
class RangeFinder {
protected:
  /// @brief Topic to which new rangefinder values should be published in meters
  /// in sensor frame.
  Core::Topic<RangeDataStamped> rangeTopic_;

public:
  /**
   * @brief Gets the rangefinder topic where new rangefinder values are
   * published in meters in sensor frame.
   * @returns rangefinder topic.
   */
  Core::Topic<RangeDataStamped> &getRangeTopic();

  /**
   * @brief Makes the sensor read the rangefinder and publish the sensor values.
   * @note Implemented by child class.
   * @return true if reading was successfull. False otherwise.
   */
  virtual bool readRange() = 0;
};

} // namespace sensor

} // namespace VCTR

#endif