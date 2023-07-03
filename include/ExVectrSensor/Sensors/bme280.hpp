
/**
 * Credit for original implementation to:
 *
 * Marshall Taylor @ SparkFun Electronics
 * May 20, 2015
 * https://github.com/sparkfun/SparkFun_BME280_Arduino_Library
 *
 * Original implementation: https://github.com/sparkfun/SparkFun_BME280_Arduino_Library
 *
 * Heavily modified for use with the ExVectrSensor library with some extra features.
 *
 *
 */

#ifndef EXVECTRSENSOR_BME280_H
#define EXVECTRSENSOR_BME280_H

#include "ExVectrCore/task_types.hpp"
#include "ExVectrCore/scheduler2.hpp"

#include "ExVectrHAL/digital_io.hpp"

#include "../barometer.hpp"

namespace VCTR
{

    namespace SNSR
    {

        class BME280 : public Barometer
        {
        public:

            static constexpr uint8_t BME280_SPI_MODE = 0;
            static constexpr size_t  BME280_SPI_CLOCK = 500000;

            static constexpr uint8_t MODE_SLEEP = 0b00;
            static constexpr uint8_t MODE_FORCED = 0b01;
            static constexpr uint8_t MODE_NORMAL = 0b11;

            // Register names:
            static constexpr uint8_t BME280_DIG_T1_LSB_REG = 0x88;
            static constexpr uint8_t BME280_DIG_T1_MSB_REG = 0x89;
            static constexpr uint8_t BME280_DIG_T2_LSB_REG = 0x8A;
            static constexpr uint8_t BME280_DIG_T2_MSB_REG = 0x8B;
            static constexpr uint8_t BME280_DIG_T3_LSB_REG = 0x8C;
            static constexpr uint8_t BME280_DIG_T3_MSB_REG = 0x8D;
            static constexpr uint8_t BME280_DIG_P1_LSB_REG = 0x8E;
            static constexpr uint8_t BME280_DIG_P1_MSB_REG = 0x8F;
            static constexpr uint8_t BME280_DIG_P2_LSB_REG = 0x90;
            static constexpr uint8_t BME280_DIG_P2_MSB_REG = 0x91;
            static constexpr uint8_t BME280_DIG_P3_LSB_REG = 0x92;
            static constexpr uint8_t BME280_DIG_P3_MSB_REG = 0x93;
            static constexpr uint8_t BME280_DIG_P4_LSB_REG = 0x94;
            static constexpr uint8_t BME280_DIG_P4_MSB_REG = 0x95;
            static constexpr uint8_t BME280_DIG_P5_LSB_REG = 0x96;
            static constexpr uint8_t BME280_DIG_P5_MSB_REG = 0x97;
            static constexpr uint8_t BME280_DIG_P6_LSB_REG = 0x98;
            static constexpr uint8_t BME280_DIG_P6_MSB_REG = 0x99;
            static constexpr uint8_t BME280_DIG_P7_LSB_REG = 0x9A;
            static constexpr uint8_t BME280_DIG_P7_MSB_REG = 0x9B;
            static constexpr uint8_t BME280_DIG_P8_LSB_REG = 0x9C;
            static constexpr uint8_t BME280_DIG_P8_MSB_REG = 0x9D;
            static constexpr uint8_t BME280_DIG_P9_LSB_REG = 0x9E;
            static constexpr uint8_t BME280_DIG_P9_MSB_REG = 0x9F;
            static constexpr uint8_t BME280_DIG_H1_REG = 0xA1;
            static constexpr uint8_t BME280_CHIP_ID_REG = 0xD0; // Chip ID
            static constexpr uint8_t BME280_RST_REG = 0xE0;     // Softreset Reg
            static constexpr uint8_t BME280_DIG_H2_LSB_REG = 0xE1;
            static constexpr uint8_t BME280_DIG_H2_MSB_REG = 0xE2;
            static constexpr uint8_t BME280_DIG_H3_REG = 0xE3;
            static constexpr uint8_t BME280_DIG_H4_MSB_REG = 0xE4;
            static constexpr uint8_t BME280_DIG_H4_LSB_REG = 0xE5;
            static constexpr uint8_t BME280_DIG_H5_MSB_REG = 0xE6;
            static constexpr uint8_t BME280_DIG_H6_REG = 0xE7;
            static constexpr uint8_t BME280_CTRL_HUMIDITY_REG = 0xF2;    // Ctrl Humidity Reg
            static constexpr uint8_t BME280_STAT_REG = 0xF3;             // Status Reg
            static constexpr uint8_t BME280_CTRL_MEAS_REG = 0xF4;        // Ctrl Measure Reg
            static constexpr uint8_t BME280_CONFIG_REG = 0xF5;           // Configuration Reg
            static constexpr uint8_t BME280_MEASUREMENTS_REG = 0xF7;     // Measurements register start
            static constexpr uint8_t BME280_PRESSURE_MSB_REG = 0xF7;     // Pressure MSB
            static constexpr uint8_t BME280_PRESSURE_LSB_REG = 0xF8;     // Pressure LSB
            static constexpr uint8_t BME280_PRESSURE_XLSB_REG = 0xF9;    // Pressure XLSB
            static constexpr uint8_t BME280_TEMPERATURE_MSB_REG = 0xFA;  // Temperature MSB
            static constexpr uint8_t BME280_TEMPERATURE_LSB_REG = 0xFB;  // Temperature LSB
            static constexpr uint8_t BME280_TEMPERATURE_XLSB_REG = 0xFC; // Temperature XLSB
            static constexpr uint8_t BME280_HUMIDITY_MSB_REG = 0xFD;     // Humidity MSB
            static constexpr uint8_t BME280_HUMIDITY_LSB_REG = 0xFE;     // Humidity LSB

        protected:
            struct BME280_SensorSettings
            {
            public:
                // Deprecated settings
                uint8_t runMode;
                uint8_t tStandby;
                uint8_t filter;
                uint8_t tempOverSample;
                uint8_t pressOverSample;
                uint8_t humidOverSample;
                float tempCorrection; // correction of temperature - added to the result
            };

            // Used to hold the calibration constants.  These are used
            // by the driver as measurements are being taking
            struct SensorCalibration
            {
            public:
                uint16_t dig_T1;
                int16_t dig_T2;
                int16_t dig_T3;

                uint16_t dig_P1;
                int16_t dig_P2;
                int16_t dig_P3;
                int16_t dig_P4;
                int16_t dig_P5;
                int16_t dig_P6;
                int16_t dig_P7;
                int16_t dig_P8;
                int16_t dig_P9;

                uint8_t dig_H1;
                int16_t dig_H2;
                uint8_t dig_H3;
                int16_t dig_H4;
                int16_t dig_H5;
                int8_t dig_H6;
            };

            struct BME280_SensorMeasurements
            {
            public:
                float temperature;
                float pressure;
                float humidity;
            };

            // settings
            BME280_SensorSettings settings;
            SensorCalibration calibration;
            int32_t t_fine;

            HAL::DigitalIO *ioBus_ = nullptr;
            bool initialised_ = false;

        public:
            // Constructor generates default BME280_SensorSettings.
            //(over-ride after construction if desired)
            BME280(void);
            //~BME280() = default;

            /**
             * @brief Initialises sensor on the given IO bus.
             * @param ioBus Input/Output bus the sensor is connected to. Should be a single connection not true bus.
             * @return true if initialisation successfull.
             */
            bool initSensor(HAL::DigitalIO &ioBus);

            /**
             * @brief Reads all data from BME280 and publishes it to topics.
             * @return true is successfull, false otherwise.
             */
            bool readSensorData();

            bool readBaro() override;

            uint8_t getMode(void);      // Get the current mode: sleep, forced, or normal
            void setMode(uint8_t mode); // Set the current mode

            void setTempOverSample(uint8_t overSampleAmount);     // Set the temperature sample mode
            void setPressureOverSample(uint8_t overSampleAmount); // Set the pressure sample mode
            void setHumidityOverSample(uint8_t overSampleAmount); // Set the humidity sample mode
            void setStandbyTime(uint8_t timeSetting);             // Set the standby time between measurements
            void setFilter(uint8_t filterSetting);                // Set the filter

            void setReferencePressure(float refPressure); // Allows user to set local sea level reference pressure
            float getReferencePressure();

            bool isMeasuring(void); // Returns true while the device is taking measurement

            // Software reset routine
            void reset(void);
            void readAllMeasurements(BME280_SensorMeasurements *measurements, uint8_t tempScale = 0);

            // Returns the values as floats.
            float readFloatPressure(void);
            float readFloatAltitudeMeters(void);
            float readFloatAltitudeFeet(void);
            void readFloatPressureFromBurst(uint8_t buffer[], BME280_SensorMeasurements *measurements);

            float readFloatHumidity(void);
            void readFloatHumidityFromBurst(uint8_t buffer[], BME280_SensorMeasurements *measurements);

            // Temperature related methods
            void setTemperatureCorrection(float corr);
            float readTempC(void);
            float readTempF(void);
            float readTempFromBurst(uint8_t buffer[]);

            double dewPointC(void);

        private:
            // The following utilities read and write

            // ReadRegisterRegion takes a uint8 array address as input and reads
            // a chunk of memory into that array.
            void readRegisterRegion(uint8_t *, uint8_t, uint8_t);
            // readRegister reads one register
            uint8_t readRegister(uint8_t);
            // Reads two regs, LSByte then MSByte order, and concatenates them
            // Used for two-byte reads
            int16_t readRegisterInt16(uint8_t offset);
            // Writes a byte;
            void writeRegister(uint8_t, uint8_t);

            uint8_t checkSampleValue(uint8_t userValue); // Checks for valid over sample values
            void readTempCFromBurst(uint8_t buffer[], BME280_SensorMeasurements *measurements);
            void readTempFFromBurst(uint8_t buffer[], BME280_SensorMeasurements *measurements);

            float _referencePressure = 101325.0; // Default but is changeable
        };


        /**
         * @brief This class uses tasks to automatically init and read the sensor. 
         */
        class BME280Driver: public BME280, public Core::Task_Periodic {
        public: 

            /**
             * @brief Constructor that uses the standard system scheduler.
             * @param ioBus The bus connection with sensor.
             */
            BME280Driver(HAL::DigitalIO &ioBus);

            /**
             * @brief Constructor that uses the given scheduler.
             * @param ioBus The bus connection with sensor.
             * @param scheduler Scheduler to use for this driver.
             */
            BME280Driver(HAL::DigitalIO &ioBus, Core::Scheduler& scheduler);

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