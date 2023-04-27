#ifndef EXVECTRSENSOR_THERMOMETER_H
#define EXVECTRSENSOR_THERMOMETER_H

#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrData/value_covariance.hpp"

namespace VCTR
{

    namespace SNSR
    {

        /**
         * @brief An abstract class for thermometers. Allows things like sensorfusion to use any thermometer class so long this is inhereted from.
         */
        class Thermometer
        {
        protected:
            /// @brief Topic to which new thermometer values should be published in degrees celsius.
            Core::Topic<Core::Timestamped<Data::ValueCov<float, 1>>> thermTopic_;

        public:
            /**
             * @brief Gets the thermometer topic where new thermometer values are published in degrees celsius.
             * @returns thermometer topic.
             */
            const Core::Topic<Core::Timestamped<Data::ValueCov<float, 1>>> &getThermoTopic() const;

            /**
             * @brief Makes the sensor read the thermometer and publish the sensor values.
             * @note Implemented by child class.
             * @return true if reading was successfull. False otherwise.
             */
            virtual bool readBaro() = 0;
        };

    }

}

#endif