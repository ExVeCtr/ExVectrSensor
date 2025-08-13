#ifndef EXVECTRSENSOR_RANGEFINDER_H
#define EXVECTRSENSOR_RANGEFINDER_H

#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrDSP/value_covariance.hpp"

namespace VCTR
{

    namespace SNSR
    {

        /**
         * @brief An abstract class for rangefinders. Allows things like sensorfusion to use any rangefinder class so long this is inhereted from.
         */
        class RangeFinder
        {
        protected:
            /// @brief Topic to which new rangefinder values should be published in meters in sensor frame.
            Core::Topic<Core::Timestamped<DSP::ValueCov<float, 1>>> rangeTopic_;

        public:
            /**
             * @brief Gets the rangefinder topic where new rangefinder values are published in meters in sensor frame.
             * @returns rangefinder topic.
             */
            Core::Topic<Core::Timestamped<DSP::ValueCov<float, 1>>> &getRangeTopic();

            /**
             * @brief Makes the sensor read the rangefinder and publish the sensor values.
             * @note Implemented by child class.
             * @return true if reading was successfull. False otherwise.
             */
            virtual bool readRange() = 0;
        };

    }

}

#endif