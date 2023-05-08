#ifndef EXVECTRSENSOR_QMC5883_H
#define EXVECTRSENSOR_QMC5883_H


#include "ExVectrHAL/io.hpp"

#include "../magnetometer.hpp"

namespace VCTR
{

    namespace SNSR
    {

        /**
         * @brief This class implements control for the QMC5883 3 axis magnetometer.
         */
        class QMC5883 : public Magnetometer
        {
        private:
            /// IO Bus to use for chip communications.
            HAL::IO *ioBus_ = nullptr;
            /// If the sensor has been successfully initialised and is in working condition
            bool initialised_ = false;

            static constexpr uint8_t QMC5883L_ADDR_DEFAULT = 0x0D;
            static constexpr uint8_t QMC5883L_X_LSB = 0;
            static constexpr uint8_t QMC5883L_X_MSB = 1;
            static constexpr uint8_t QMC5883L_Y_LSB = 2;
            static constexpr uint8_t QMC5883L_Y_MSB = 3;
            static constexpr uint8_t QMC5883L_Z_LSB = 4;
            static constexpr uint8_t QMC5883L_Z_MSB = 5;
            static constexpr uint8_t QMC5883L_STATUS = 6;
            static constexpr uint8_t QMC5883L_TEMP_LSB = 7;
            static constexpr uint8_t QMC5883L_TEMP_MSB = 8;
            static constexpr uint8_t QMC5883L_CONFIG = 9;
            static constexpr uint8_t QMC5883L_CONFIG2 = 10;
            static constexpr uint8_t QMC5883L_RESET = 11;
            static constexpr uint8_t QMC5883L_RESERVED = 12;
            static constexpr uint8_t QMC5883L_CHIP_ID = 13;

        public:
        
            /**
             * @brief Initialises and sets sensors settings.
             * @param ioBus Which bus to use for communications.
             * @return true if successfull and sensor is running, false otherwise.
             */
            bool initSensor(HAL::IO &ioBus);

            /**
             * @brief Makes the sensor read the values and publish them to the topic.
             * @note Implemented by child class.
             * @return true if reading was successfull. False otherwise.
             */
            bool readMag() override;

            /**
             * @brief The lowest interval to read the magnetometer at.
             * @return Interval in nanoseconds
             */
            int64_t getMagInterval() const override;

        private:
            /**
             * @brief Checks if data is currently available to read.
             * @return true if available
             */
            bool dataAvailable();

        };

    }

}

#endif