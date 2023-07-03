#ifndef EXVECTRSENSOR_ENCODER_H
#define EXVECTRSENSOR_ENCODER_H

#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrData/value_covariance.hpp"

namespace VCTR
{

    namespace SNSR
    {

        /**
         * @brief An abstract class for encoders. Allows things like sensorfusion to use any encoder class so long this is inhereted from.
         * @note Publishing a value to the encoder topic will set the encoder to the value.
         */
        class Encoder
        {
        protected:
            /// @brief Topic to which new encoder values should be published in counts.
            Core::Topic<Core::Timestamped<Data::ValueCov<int32_t, 1>>> encoderTopic_;

        public:
            /**
             * @brief Gets the encoder topic where new encoder values are published in counts.
             * @returns encoder topic.
             */
            Core::Topic<Core::Timestamped<Data::ValueCov<int32_t, 1>>> &getEncoderTopic();

            /**
             * @brief Makes the sensor read the values and publish them to the topic.
             * @note Implemented by child class.
             * @return true if reading was successfull. False otherwise.
             */
            virtual bool readEncoder() = 0;
        };

    }

}

#endif