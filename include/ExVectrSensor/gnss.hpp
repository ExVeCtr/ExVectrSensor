#ifndef EXVECTRSENSOR_GNSS_H
#define EXVECTRSENSOR_GNSS_H

#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrDSP/value_covariance.hpp"

namespace VCTR
{

    namespace SNSR
    {

        /**
         * Simple container class for containing GNSS data. This should be used as its designed to maximise position accuracy (double) and reduce memory usage.
         * @note The position and velocity are in NED frame. 
         */
        struct GNSSData
        {
        public:

            /// @brief in form [latitude, longitude, altitude] lat, lon in radians altitude in meters above ellipsoid.
            Math::Vector_D position = 0;
            Math::Vector_F velocity = 0;

            Math::Vector_F positionCov = 1;
            Math::Vector_F velocityCov = 1;

            bool positionValid = false;
            bool velocityValid = false;

            uint8_t numSats = 0;

        };


        /**
         * @brief An abstract class for GNSS receivers. Allows things like sensorfusion to use any GNSS class so long this is inhereted from.
         */
        class GNSS
        {
        protected:
            /// @brief Topic to which new accelerometer values should be published in [m/s/s] in sensor frame.
            Core::Topic<Core::Timestamped<SNSR::GNSSData>> gnssTopic_;

        public:
            /**
             * @brief Gets the GNSS topic where new GNSS positions/velocities are published.
             * @returns GNSS topic.
             */
            Core::Topic<Core::Timestamped<SNSR::GNSSData>> &getGNSSTopic();

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