#ifndef EXVECTRSENSOR_BAROMETER_H
#define EXVECTRSENSOR_BAROMETER_H

#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrData/value_covariance.hpp"

namespace VCTR
{

    namespace SNSR
    {

        /**
         * @brief An abstract class for barometers. Allows things like sensorfusion to use any barometer class so long this is inhereted from.
         */
        class Barometer
        {
        protected:
            /// @brief Topic to which new barometer values should be published in Pascal in sensor frame.
            Core::Topic<Core::Timestamped<Data::ValueCov<float, 1>>> baroTopic_;

        public:
            /**
             * @brief Gets the barometer topic where new barometer values are published in Pascal in sensor frame.
             * @returns barometer topic.
             */
            const Core::Topic<Core::Timestamped<Data::ValueCov<float, 1>>> &getBaroTopic() const;

            /**
             * @brief Makes the sensor read the barometer and publish the sensor values.
             * @note Implemented by child class.
             * @return true if reading was successfull. False otherwise.
             */
            virtual bool readBaro() = 0;

            /**
             * @brief The lowest interval to read the barometer at.
             * @return Interval in nanoseconds.
             */
            virtual int64_t getBaroInterval() const = 0;
        };

    }

}

#endif