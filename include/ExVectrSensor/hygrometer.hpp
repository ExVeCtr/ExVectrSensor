#ifndef EXVECTRSENSOR_hygrometer_H
#define EXVECTRSENSOR_hygrometer_H

#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrDSP/value_covariance.hpp"

namespace VCTR
{

    namespace SNSR
    {

        /**
         * @brief An abstract class for hygrometers. Allows things like sensorfusion to use any hygrometer class so long this is inhereted from.
         */
        class Hygrometer
        {
        protected:
            /// @brief Topic to which new hygrometer values should be published in degrees celsius.
            Core::Topic<Core::Timestamped<DSP::ValueCov<float, 1>>> hygroTopic_;

        public:
            /**
             * @brief Gets the hygrometer topic where new hygrometer values are published in degrees celsius.
             * @returns hygrometer topic.
             */
            Core::Topic<Core::Timestamped<DSP::ValueCov<float, 1>>> &getHygroTopic();

            /**
             * @brief Makes the sensor read the hygrometer and publish the sensor values.
             * @note Implemented by child class.
             * @return true if reading was successfull. False otherwise.
             */
            virtual bool readHygro() = 0;
        };

    }

}

#endif