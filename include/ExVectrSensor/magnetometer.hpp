#ifndef EXVECTRSENSOR_MAGNETOMETER_H
#define EXVECTRSENSOR_MAGNETOMETER_H

#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrData/value_covariance.hpp"

namespace VCTR
{

    namespace SNSR
    {

        /**
         * @brief An abstract class for magnetometers. Allows things like sensorfusion to use any magnetometer class so long this is inhereted from.
         */
        class Magnetometer
        {
        protected:
            /// @brief Topic to which new magnetometer values should be published in [tesla] in sensor frame.
            Core::Topic<Core::Timestamped<Data::ValueCov<float, 3>>> magTopic_;

        public:
            /**
             * @brief Gets the magnetometer topic where new magnetometer values are published in [tesla] in sensor frame.
             * @returns magnetometer topic.
             */
            const Core::Topic<Core::Timestamped<Data::ValueCov<float, 3>>> &getMagTopic() const;

            /**
             * @brief Makes the sensor read the values and publish them to the topic.
             * @note Implemented by child class.
             * @return true if reading was successfull. False otherwise.
             */
            virtual bool readMag() = 0;
        };

    }

}

#endif