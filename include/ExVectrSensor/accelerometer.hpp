#ifndef EXVECTRSENSOR_ACCELEROMETER_H
#define EXVECTRSENSOR_ACCELEROMETER_H

#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrData/value_covariance.hpp"

namespace VCTR
{

    namespace SNSR
    {

        /**
         * @brief An abstract class for accelerometers. Allows things like sensorfusion to use any accelerometer class so long this is inhereted from.
         */
        class Accelerometer
        {
        protected:
            /// @brief Topic to which new accelerometer values should be published in [m/s/s] in sensor frame.
            Core::Topic<Core::Timestamped<Data::ValueCov<float, 3>>> accelTopic_;

        public:
            /**
             * @brief Gets the accelerometer topic where new accelerometer values are published in [m/s/s] in sensor frame.
             * @returns accelerometer topic.
             */
            Core::Topic<Core::Timestamped<Data::ValueCov<float, 3>>> &getAccelTopic();

            /**
             * @brief Makes the sensor read the values and publish them to the topic.
             * @note Implemented by child class.
             * @return true if reading was successfull. False otherwise.
             */
            virtual bool readAccel() = 0;
        };

    }

}

#endif