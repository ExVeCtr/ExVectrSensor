#ifndef EXVECTRSENSOR_GYROSCOPE_H
#define EXVECTRSENSOR_GYROSCOPE_H

#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrData/value_covariance.hpp"

namespace VCTR
{

    namespace SNSR
    {

        /**
         * @brief An abstract class for gyroscopes. Allows things like sensorfusion to use any gyroscope class so long this is inhereted from.
         */
        class Gyroscope
        {
        protected:
            /// @brief Topic to which new gyroscope values should be published in [rad/s] in sensor frame.
            Core::Topic<Core::Timestamped<Data::ValueCov<float, 3>>> gyroTopic_;

        public:
            /**
             * @brief Gets the gyroscope topic where new gyroscope values are published in [rad/s] in sensor frame.
             * @returns gyroscope topic.
             */
            const Core::Topic<Core::Timestamped<Data::ValueCov<float, 3>>> &getGyroTopic() const;

            /**
             * @brief Makes the sensor read the values and publish them to the topic.
             * @note Implemented by child class.
             * @return true if reading was successfull. False otherwise.
             */
            virtual bool readGyro() = 0;
        };

    }

}

#endif