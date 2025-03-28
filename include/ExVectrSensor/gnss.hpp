#ifndef EXVECTRSENSOR_GNSS_H
#define EXVECTRSENSOR_GNSS_H

#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrData/value_covariance.hpp"
#include "ExVectrData/gnss_data.hpp"

namespace VCTR
{

    namespace SNSR
    {

        /**
         * @brief An abstract class for GNSS receivers. Allows things like sensorfusion to use any GNSS class so long this is inhereted from.
         */
        class GNSS
        {
        protected:
            /// @brief Topic to which new accelerometer values should be published in [m/s/s] in sensor frame.
            Core::Topic<Core::Timestamped<Data::GNSSData>> gnssTopic_;

        public:
            /**
             * @brief Gets the GNSS topic where new GNSS positions/velocities are published.
             * @returns GNSS topic.
             */
            Core::Topic<Core::Timestamped<Data::GNSSData>> &getGNSSTopic();

            /**
             * @brief Makes the sensor read the values and publish them to the topic.
             * @note Implemented by child class.
             * @return true if reading was successfull. False otherwise.
             */
            virtual bool readGNSS() = 0;
        };

    }

}

#endif