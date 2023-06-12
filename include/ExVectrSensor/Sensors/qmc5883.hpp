#ifndef EXVECTRSENSOR_QMC5883_H
#define EXVECTRSENSOR_QMC5883_H

#include "ExVectrCore/task_types.hpp"
#include "ExVectrCore/scheduler2.hpp"

#include "ExVectrHAL/digital_io.hpp"

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
        protected:
            /// IO Bus to use for chip communications.
            HAL::DigitalIO *ioBus_ = nullptr;
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
            bool initSensor(HAL::DigitalIO &ioBus);

            /**
             * @brief Makes the sensor read the values and publish them to the topic.
             * @note Implemented by child class.
             * @return true if reading was successfull. False otherwise.
             */
            bool readMag() override;

        private:
            /**
             * @brief Checks if data is currently available to read.
             * @return true if available
             */
            bool dataAvailable();
        };

        /**
         * @brief This class uses tasks to automatically init and read the sensor.
         */
        class QMC5883Driver : public QMC5883, public Core::Task_Periodic
        {
        public:
            /**
             * @brief Constructor that uses the standard system scheduler.
             * @param ioBus The bus connection with sensor.
             */
            QMC5883Driver(HAL::DigitalIO &ioBus);

            /**
             * @brief Constructor that uses the given scheduler.
             * @param ioBus The bus connection with sensor.
             * @param scheduler Scheduler to use for this driver.
             */
            QMC5883Driver(HAL::DigitalIO &ioBus, Core::Scheduler &scheduler);

            /**
             * @brief Initialises sensor and expected to be called once at start by scheduler
             */
            void taskInit() override;

            /**
             * @brief main task thread that reads all sensor data and publishes it. To be called by scheduler.
             */
            void taskThread() override;
        };

    }

}

#endif